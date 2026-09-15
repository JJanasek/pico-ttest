/**
 * @file tvla_target.cpp
 * @brief Host-driven TVLA capture target for TROPIC01 on ESP32.
 *
 * Built by the `esp32dev_tvla` environment only (see platformio.ini); the default
 * `esp32dev` environment still builds main.cpp and is untouched.
 *
 * Unlike main.cpp - which runs a self-contained software (cycle-count) TVLA demo - this
 * firmware does nothing on its own. It waits for line commands on the serial port and
 * performs one signature per command, raising TVLA_TRIGGER_PIN for exactly the duration of
 * the libtropic sign call so an oscilloscope can capture that window. The fixed-vs-random
 * class split lives on the host (tvla_capture.py), which also stores the traces.
 *
 * Protocol (ASCII, '\n' terminated, host -> target):
 *   p                 ping                       -> +PONG
 *   v                 version                    -> +VERSION tvla-target 3
 *   g ed | g ec       erase slot + generate key  -> +OK <pubkey hex>
 *   k ed|ec <hex>     erase slot + store the given 32 byte private key (secret scalar), so the
 *                     host can run a fixed-vs-random *scalar* campaign  -> +OK
 *   s <hex>           sign payload with trigger  -> +OK <signature hex> <l3 nonce hex>
 *   e                 erase the active slot      -> +OK
 * Replies: "+..." success, "-ERR ..." failure, "#..." informational/log (ignore on the host).
 */

#include <Arduino.h>
#include <SPI.h>

#include <LibtropicArduino.h>
// MbedTLS's PSA Crypto library. libtropic's mbedtls_v4 CAL calls into PSA but does not initialize
// it, so - exactly as in main.cpp and the upstream examples - psa_crypto_init() must run first or
// the secure channel handshake fails.
#include "psa/crypto.h"

// ------------------------------------ Configuration -------------------------------------
// Platform's pin number where TROPIC01's SPI Chip Select pin is connected.
#define TROPIC01_CS_PIN 5
#if LT_USE_INT_PIN
// Platform's pin number where TROPIC01's interrupt pin is connected.
#define TROPIC01_INT_PIN 4
#endif

// GPIO raised for the duration of the traced sign call; wire it to the scope trigger input:
// ChipWhisperer-Husky TIO4 (20-pin header pin 16), or a PicoScope channel. Overridable from
// platformio.ini build_flags.
#ifndef TVLA_TRIGGER_PIN
#define TVLA_TRIGGER_PIN 4
#endif

// GPIO4 is only free because LT_USE_INT_PIN is off in this project; with it on, that pin is
// TROPIC01's interrupt input. Fail the build rather than let the two drive the same line.
#if LT_USE_INT_PIN && (TVLA_TRIGGER_PIN == TROPIC01_INT_PIN)
#error "TVLA_TRIGGER_PIN collides with TROPIC01_INT_PIN - pick a different trigger GPIO"
#endif

// Serial line speed. Must match monitor_speed / the host's --baud.
#ifndef TVLA_SERIAL_BAUD
#define TVLA_SERIAL_BAUD 115200
#endif

// Pairing Key macros for establishing a Secure Channel Session with TROPIC01.
// Using the default Pairing Key slot 0 of Production TROPIC01 chips.
#define PAIRING_KEY_PRIV lt_sh0priv_prod0
#define PAIRING_KEY_PUB lt_sh0pub_prod0
#define PAIRING_KEY_SLOT TR01_PAIRING_KEY_SLOT_INDEX_0

// One slot per curve, so switching curves does not require re-provisioning the other one.
#define ECC_SLOT_ED25519 TR01_ECC_SLOT_1
#define ECC_SLOT_ECDSA TR01_ECC_SLOT_2

// Longest payload accepted by `s`. ECDSA needs exactly 32 bytes (a message hash), EdDSA takes
// an arbitrary message; 256 is plenty for a fixed-vs-random campaign and keeps the line buffer small.
#define TVLA_MAX_MSG_LEN 256
#define TVLA_LINE_LEN (2 * TVLA_MAX_MSG_LEN + 16)

#define TR01_PUBKEY_MAX_LEN 64  // 32B for Ed25519, 64B for P256

// lt_ecc_ecdsa_sign() requires the hash to be exactly TR01_L3_ECDSA_SIGN_CMD_MSG_HASH_LEN bytes.
// That macro lives in libtropic's private lt_l3_api_structs.h, which is not on the app include
// path, so mirror its value here.
#define TVLA_ECDSA_HASH_LEN 32
// ----------------------------------------------------------------------------------------

#if LT_SEPARATE_L3_BUFF
// User's own buffer for L3 Layer data.
uint8_t l3_buffer[LT_SIZE_OF_L3_BUFF] __attribute__((aligned(16))) = {0};
#endif

// TROPIC01 instance. The directives mirror main.cpp so this file stays functional with every
// supported Libtropic CMake option.
Tropic01 tropic01(TROPIC01_CS_PIN
#if LT_USE_INT_PIN
                  ,
                  TROPIC01_INT_PIN
#endif
#if LT_SEPARATE_L3_BUFF
                  ,
                  l3_buffer, sizeof(l3_buffer)
#endif
);

// ------------------------------ Secure-channel nonce access -----------------------------
// libtropic maintains an L3 nonce - the AES-GCM IV used to encrypt each L3 command
// (handle.l3.encryption_IV): zeroed at session start, incremented by one per command. The value
// in effect when a Sign command is encrypted is what a white-box analysis needs per trace, since
// it differs between traces even when the message is identical.
//
// The Arduino wrapper keeps its lt_handle_t private with no accessor, so we reach it with the
// standard explicit-instantiation access idiom (legal C++: an explicit template instantiation is
// exempt from access checks). If libtropic-arduino gains a public accessor, replace this with it.
namespace {
template <typename Tag, typename Tag::type M>
struct PrivateRob {
    friend typename Tag::type robbed(Tag) { return M; }
};
struct Tropic01HandleTag {
    typedef lt_handle_t Tropic01::*type;
    friend type robbed(Tropic01HandleTag);
};
template struct PrivateRob<Tropic01HandleTag, &Tropic01::handle>;

// Pointer to the 12-byte L3 IV (the secure-channel nonce) currently in the handle.
static const uint8_t *secureChannelNonce(void)
{
    lt_handle_t &h = tropic01.*robbed(Tropic01HandleTag());
    return h.l3.encryption_IV;
}
}  // namespace

// Curve currently provisioned by the last successful `g` command.
static lt_ecc_curve_type_t activeCurve = TR01_CURVE_ED25519;
static lt_ecc_slot_t activeSlot = ECC_SLOT_ED25519;
static bool keyProvisioned = false;

static char lineBuf[TVLA_LINE_LEN];
static uint8_t msgBuf[TVLA_MAX_MSG_LEN];
static uint8_t sigBuf[TR01_ECDSA_EDDSA_SIGNATURE_LENGTH];
static uint8_t pubKeyBuf[TR01_PUBKEY_MAX_LEN];

// -------------------------------------- Helpers -----------------------------------------
static void replyError(const char msg[], const lt_ret_t ret)
{
    Serial.print("-ERR ");
    Serial.print(msg);
    Serial.print(" ret=");
    Serial.print(ret);
    Serial.print(" (");
    Serial.print(lt_ret_verbose(ret));
    Serial.println(")");
}

static void replyHex(const char prefix[], const uint8_t data[], const size_t len)
{
    Serial.print(prefix);
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(data[i], HEX);
    }
    Serial.println();
}

static int hexNibble(const char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Decodes `hex` into `out`. Returns the byte count, or -1 on malformed / oversized input.
static int hexDecode(const char hex[], uint8_t out[], const size_t outMaxLen)
{
    const size_t hexLen = strlen(hex);
    if ((hexLen % 2) != 0 || (hexLen / 2) > outMaxLen) {
        return -1;
    }
    for (size_t i = 0; i < hexLen; i += 2) {
        const int hi = hexNibble(hex[i]);
        const int lo = hexNibble(hex[i + 1]);
        if (hi < 0 || lo < 0) {
            return -1;
        }
        out[i / 2] = (uint8_t)((hi << 4) | lo);
    }
    return (int)(hexLen / 2);
}

// Blocking read of one '\n' terminated line into lineBuf. '\r' and leading spaces are stripped.
// Returns the line length; oversized lines are truncated (the remainder is discarded).
static size_t readLine(void)
{
    size_t len = 0;
    bool overflow = false;

    while (true) {
        while (!Serial.available());
        const int c = Serial.read();
        if (c < 0 || c == '\r') {
            continue;
        }
        if (c == '\n') {
            break;
        }
        if (len == 0 && c == ' ') {
            continue;
        }
        if (len < sizeof(lineBuf) - 1) {
            lineBuf[len++] = (char)c;
        }
        else {
            overflow = true;
        }
    }

    lineBuf[len] = '\0';
    if (overflow) {
        Serial.println("# line too long, truncated");
    }
    return len;
}

// Returns the argument of a command line, i.e. everything past the command letter and spaces.
static const char *lineArgument(void)
{
    const char *arg = lineBuf + 1;
    while (*arg == ' ') {
        arg++;
    }
    return arg;
}
// ----------------------------------------------------------------------------------------

// ---------------------------------- Command handlers ------------------------------------
// `g ed` / `g ec`: erase the curve's slot, generate a fresh key in it and return the public key.
static void handleKeyGenerate(void)
{
    const char *arg = lineArgument();
    lt_ecc_curve_type_t curve;
    lt_ecc_slot_t slot;
    size_t pubKeyLen;

    if (strcmp(arg, "ed") == 0) {
        curve = TR01_CURVE_ED25519;
        slot = ECC_SLOT_ED25519;
        pubKeyLen = TR01_CURVE_ED25519_PUBKEY_LEN;
    }
    else if (strcmp(arg, "ec") == 0) {
        curve = TR01_CURVE_P256;
        slot = ECC_SLOT_ECDSA;
        pubKeyLen = TR01_CURVE_P256_PUBKEY_LEN;
    }
    else {
        Serial.println("-ERR unknown curve, expected 'ed' or 'ec'");
        return;
    }

    // Erase first so the generate does not fail on an already written slot.
    lt_ret_t ret = tropic01.eccKeyErase(slot);
    if (ret != LT_OK) {
        replyError("eccKeyErase() failed", ret);
        return;
    }

    ret = tropic01.eccKeyGenerate(slot, curve);
    if (ret != LT_OK) {
        replyError("eccKeyGenerate() failed", ret);
        return;
    }

    lt_ecc_curve_type_t readCurve;
    lt_ecc_key_origin_t readOrigin;
    ret = tropic01.eccKeyRead(slot, pubKeyBuf, sizeof(pubKeyBuf), readCurve, readOrigin);
    if (ret != LT_OK) {
        replyError("eccKeyRead() failed", ret);
        return;
    }

    activeCurve = curve;
    activeSlot = slot;
    keyProvisioned = true;

    replyHex("+OK ", pubKeyBuf, pubKeyLen);
}

// `k ed <hex>` / `k ec <hex>`: erase the curve's slot and store the given private key in it.
// Used by the fixed-vs-random *scalar* campaign: the host writes a fresh secret scalar before
// each trace. The store happens outside the trigger window, so only the signature is captured.
static void handleKeyStore(void)
{
    const char *arg = lineArgument();
    lt_ecc_curve_type_t curve;
    lt_ecc_slot_t slot;

    if (strncmp(arg, "ed", 2) == 0) {
        curve = TR01_CURVE_ED25519;
        slot = ECC_SLOT_ED25519;
    }
    else if (strncmp(arg, "ec", 2) == 0) {
        curve = TR01_CURVE_P256;
        slot = ECC_SLOT_ECDSA;
    }
    else {
        Serial.println("-ERR unknown curve, expected 'ed' or 'ec'");
        return;
    }

    const char *hex = arg + 2;
    while (*hex == ' ') {
        hex++;
    }

    const int keyLen = hexDecode(hex, msgBuf, sizeof(msgBuf));
    if (keyLen != TR01_CURVE_PRIVKEY_LEN) {
        Serial.print("-ERR private key must be ");
        Serial.print(TR01_CURVE_PRIVKEY_LEN);
        Serial.println(" bytes");
        return;
    }

    lt_ret_t ret = tropic01.eccKeyErase(slot);
    if (ret != LT_OK) {
        replyError("eccKeyErase() failed", ret);
        return;
    }

    ret = tropic01.eccKeyStore(slot, curve, msgBuf);
    if (ret != LT_OK) {
        replyError("eccKeyStore() failed", ret);
        return;
    }

    activeCurve = curve;
    activeSlot = slot;
    keyProvisioned = true;

    Serial.println("+OK");
}

// `s <hex>`: sign the payload with the provisioned key, trigger asserted around the call only.
static void handleSign(void)
{
    if (!keyProvisioned) {
        Serial.println("-ERR no key provisioned, send 'g ed' or 'g ec' first");
        return;
    }

    const int msgLen = hexDecode(lineArgument(), msgBuf, sizeof(msgBuf));
    if (msgLen <= 0) {
        Serial.println("-ERR malformed hex payload");
        return;
    }

    // The nonce that will encrypt this Sign command: read the IV now, before the call increments
    // it. This happens outside the trigger window, so it adds nothing to the captured trace.
    uint8_t nonceUsed[TR01_L3_IV_SIZE];
    memcpy(nonceUsed, secureChannelNonce(), sizeof(nonceUsed));

    lt_ret_t ret;
    if (activeCurve == TR01_CURVE_ED25519) {
        digitalWrite(TVLA_TRIGGER_PIN, HIGH);
        ret = tropic01.eddsaSign(activeSlot, msgBuf, (uint16_t)msgLen, sigBuf);
        digitalWrite(TVLA_TRIGGER_PIN, LOW);
    }
    else {
        if (msgLen != TVLA_ECDSA_HASH_LEN) {
            Serial.println("-ERR ECDSA payload must be a 32 byte hash");
            return;
        }
        digitalWrite(TVLA_TRIGGER_PIN, HIGH);
        ret = tropic01.ecdsaSign(activeSlot, msgBuf, (uint32_t)msgLen, sigBuf);
        digitalWrite(TVLA_TRIGGER_PIN, LOW);
    }

    if (ret != LT_OK) {
        replyError("sign failed", ret);
        return;
    }

    // Reply: +OK <signature hex> <nonce hex>. The nonce is the 12-byte L3 IV used for this command.
    Serial.print("+OK ");
    for (size_t i = 0; i < sizeof(sigBuf); i++) {
        if (sigBuf[i] < 0x10) Serial.print("0");
        Serial.print(sigBuf[i], HEX);
    }
    Serial.print(" ");
    for (size_t i = 0; i < sizeof(nonceUsed); i++) {
        if (nonceUsed[i] < 0x10) Serial.print("0");
        Serial.print(nonceUsed[i], HEX);
    }
    Serial.println();
}

// `e`: erase the active slot, e.g. to leave the chip clean at the end of a campaign.
static void handleErase(void)
{
    const lt_ret_t ret = tropic01.eccKeyErase(activeSlot);
    if (ret != LT_OK) {
        replyError("eccKeyErase() failed", ret);
        return;
    }
    keyProvisioned = false;
    Serial.println("+OK");
}
// ----------------------------------------------------------------------------------------

void setup()
{
    pinMode(TVLA_TRIGGER_PIN, OUTPUT);
    digitalWrite(TVLA_TRIGGER_PIN, LOW);

    SPI.begin();

    Serial.begin(TVLA_SERIAL_BAUD);
    while (!Serial);  // Wait for serial port to connect.

    Serial.println("# TROPIC01 TVLA capture target");
    Serial.print("# trigger pin: ");
    Serial.println(TVLA_TRIGGER_PIN);

    const psa_status_t psaStatus = psa_crypto_init();
    if (psaStatus != PSA_SUCCESS) {
        Serial.print("-ERR psa_crypto_init() failed, psa_status_t=");
        Serial.println(psaStatus);
        while (true);
    }

    lt_ret_t ret = tropic01.begin();
    if (ret != LT_OK) {
        replyError("Tropic01.begin() failed", ret);
        while (true);
    }

    ret = tropic01.secureSessionStart(PAIRING_KEY_PRIV, PAIRING_KEY_PUB, PAIRING_KEY_SLOT);
    if (ret != LT_OK) {
        replyError("Tropic01.secureSessionStart() failed", ret);
        while (true);
    }

    // The host waits for this line before sending commands.
    Serial.println("+READY");
}

void loop()
{
    if (readLine() == 0) {
        return;
    }

    switch (lineBuf[0]) {
        case 'p':
            Serial.println("+PONG");
            break;
        case 'v':
            Serial.println("+VERSION tvla-target 3");
            break;
        case 'g':
            handleKeyGenerate();
            break;
        case 'k':
            handleKeyStore();
            break;
        case 's':
            handleSign();
            break;
        case 'e':
            handleErase();
            break;
        default:
            Serial.println("-ERR unknown command");
            break;
    }
}
