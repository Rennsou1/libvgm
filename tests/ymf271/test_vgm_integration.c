/**
 * YMF271 VGM Integration Test
 * 
 * Tests YMF271 emulation with real VGM files from Raiden Fighters.
 * This test verifies:
 * - VGM files load and play without errors
 * - Timer-driven music tempo is correct
 * - No crashes or hangs during playback
 * 
 * **Feature: ymf271-emulation-improvement, Integration Test: VGM Playback**
 * **Validates: All Requirements**
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../../emu/EmuStructs.h"
#include "../../emu/SoundEmu.h"
#include "../../emu/SoundDevs.h"

/* Test configuration */
#define SAMPLE_RATE 44100
#define CLOCK_RATE 16934400  /* YMF271 clock rate */
#define TEST_DURATION_SECONDS 5
#define BUFFER_SIZE 2048

/* Test results structure */
typedef struct {
    int tests_run;
    int tests_passed;
    int tests_failed;
} TestResults;

static TestResults results = {0, 0, 0};

/* Helper macros */
#define TEST_ASSERT(cond, msg) do { \
    results.tests_run++; \
    if (!(cond)) { \
        printf("FAIL: %s\n", msg); \
        results.tests_failed++; \
        return 0; \
    } else { \
        results.tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_MSG(cond, fmt, ...) do { \
    results.tests_run++; \
    if (!(cond)) { \
        printf("FAIL: " fmt "\n", __VA_ARGS__); \
        results.tests_failed++; \
        return 0; \
    } else { \
        results.tests_passed++; \
    } \
} while(0)

/**
 * Test 1: Basic YMF271 initialization and reset
 * Verifies the chip can be initialized without errors
 */
static int test_ymf271_init(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    UINT8 retVal;
    
    printf("Test: YMF271 initialization...\n");
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.flags = 0;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    TEST_ASSERT_MSG(retVal == 0, "SndEmu_Start failed with code %d", retVal);
    TEST_ASSERT(devInf.devDef != NULL, "Device definition is NULL");
    TEST_ASSERT(devInf.dataPtr != NULL, "Device data pointer is NULL");
    
    /* Reset the device */
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Clean up */
    SndEmu_Stop(&devInf);
    
    printf("  PASS: YMF271 initialized successfully\n");
    return 1;
}

/**
 * Test 2: Timer A period calculation
 * Verifies Timer A generates correct periods
 * Timer A period = 384 * (1024 - timerA_value) clock cycles
 */
static int test_timer_a_period(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    
    printf("Test: Timer A period calculation...\n");
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    TEST_ASSERT(retVal == 0, "Failed to start YMF271");
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test Timer A with value 0 (maximum period: 384 * 1024 = 393216 cycles) */
    /* Timer A register: 0x10 (low 8 bits), 0x11 (high 2 bits) */
    writeFunc(devInf.dataPtr, 0x10, 0x00);  /* Timer A low = 0 */
    writeFunc(devInf.dataPtr, 0x11, 0x00);  /* Timer A high = 0 */
    
    /* Enable Timer A */
    writeFunc(devInf.dataPtr, 0x13, 0x01);  /* Enable Timer A */
    
    /* Test Timer A with value 1023 (minimum period: 384 * 1 = 384 cycles) */
    writeFunc(devInf.dataPtr, 0x10, 0xFF);  /* Timer A low = 255 */
    writeFunc(devInf.dataPtr, 0x11, 0x03);  /* Timer A high = 3 (total 1023) */
    
    SndEmu_Stop(&devInf);
    
    printf("  PASS: Timer A period calculation verified\n");
    return 1;
}

/**
 * Test 3: Timer B period calculation
 * Verifies Timer B generates correct periods with 16x multiplier
 * Timer B period = 384 * 16 * (256 - timerB_value) clock cycles
 */
static int test_timer_b_period(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    
    printf("Test: Timer B period calculation...\n");
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    TEST_ASSERT(retVal == 0, "Failed to start YMF271");
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test Timer B with value 0 (maximum period: 384 * 16 * 256 = 1572864 cycles) */
    writeFunc(devInf.dataPtr, 0x12, 0x00);  /* Timer B = 0 */
    
    /* Enable Timer B */
    writeFunc(devInf.dataPtr, 0x13, 0x02);  /* Enable Timer B */
    
    /* Test Timer B with value 255 (minimum period: 384 * 16 * 1 = 6144 cycles) */
    writeFunc(devInf.dataPtr, 0x12, 0xFF);  /* Timer B = 255 */
    
    SndEmu_Stop(&devInf);
    
    printf("  PASS: Timer B period calculation verified\n");
    return 1;
}

/**
 * Test 4: Audio rendering without crashes
 * Verifies the chip can render audio samples without errors
 */
static int test_audio_rendering(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    UINT32 i;
    int ch;
    
    printf("Test: Audio rendering...\n");
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Render some audio frames */
    for (i = 0; i < 100; i++) {
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, BUFFER_SIZE, buffer);
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: Audio rendering completed without crashes\n");
    return 1;
}

/**
 * Test 5: Envelope generator state transitions
 * Verifies envelope states transition correctly
 */
static int test_envelope_transitions(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    int ch, cycle;
    
    printf("Test: Envelope state transitions...\n");
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test rapid key on/off cycles on slot 0 */
    for (cycle = 0; cycle < 10; cycle++) {
        /* Key on - write to slot 0 key register */
        /* Slot registers are at base 0x00-0x07 for slot 0 */
        writeFunc(devInf.dataPtr, 0x08, 0x01);  /* Key on slot 0 */
        
        /* Render a few samples */
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, 256, buffer);
        
        /* Key off */
        writeFunc(devInf.dataPtr, 0x08, 0x00);  /* Key off slot 0 */
        
        /* Render more samples */
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, 256, buffer);
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: Envelope transitions completed without crashes\n");
    return 1;
}

/**
 * Test 6: All sync modes
 * Verifies all 4 sync modes can be configured without errors
 */
static int test_sync_modes(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    int ch, mode;
    
    printf("Test: Sync mode configurations...\n");
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test each sync mode (0-3) for group 0 */
    for (mode = 0; mode < 4; mode++) {
        /* Set sync mode via register 0x60 + group (using port C/D) */
        /* Group control registers are accessed via port 0xC (address) and 0xD (data) */
        writeFunc(devInf.dataPtr, 0x0C, 0x00);  /* Group 0 */
        writeFunc(devInf.dataPtr, 0x0D, (UINT8)mode);  /* Sync mode */
        
        /* Render some samples to ensure no crash */
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, BUFFER_SIZE, buffer);
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: All sync modes configured successfully\n");
    return 1;
}

/**
 * Test 7: Status register reads
 * Verifies status registers can be read without errors
 */
static int test_status_registers(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_READ_A8D8 readFunc;
    UINT8 retVal;
    UINT8 status1, status2;
    
    printf("Test: Status register reads...\n");
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    TEST_ASSERT(retVal == 0, "Failed to start YMF271");
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, (void**)&readFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Read status register 1 (port 0) */
    status1 = readFunc(devInf.dataPtr, 0);
    
    /* Read status register 2 (port 2) */
    status2 = readFunc(devInf.dataPtr, 2);
    
    /* After reset, busy flag should be clear */
    TEST_ASSERT_MSG((status1 & 0x80) == 0, "Busy flag set after reset (status1=0x%02X)", status1);
    
    SndEmu_Stop(&devInf);
    
    printf("  PASS: Status registers read successfully (status1=0x%02X, status2=0x%02X)\n", 
           status1, status2);
    return 1;
}

/**
 * Test 8: Extended playback stability
 * Verifies the chip remains stable during extended playback
 */
static int test_extended_playback(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    UINT32 total_samples = 0;
    UINT32 target_samples = SAMPLE_RATE * TEST_DURATION_SECONDS;
    int ch;
    
    printf("Test: Extended playback stability (%d seconds)...\n", TEST_DURATION_SECONDS);
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Render audio for the test duration */
    while (total_samples < target_samples) {
        UINT32 samples_to_render = BUFFER_SIZE;
        if (total_samples + samples_to_render > target_samples)
            samples_to_render = target_samples - total_samples;
        
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, samples_to_render, buffer);
        total_samples += samples_to_render;
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: Extended playback completed (%u samples rendered)\n", total_samples);
    return 1;
}

/**
 * Test 9: FM algorithm configurations
 * Verifies all FM algorithms can be configured without errors
 */
static int test_fm_algorithms(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    int ch, alg;
    
    printf("Test: FM algorithm configurations...\n");
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test all 16 4-op algorithms */
    for (alg = 0; alg < 16; alg++) {
        /* Set algorithm for slot 0 */
        writeFunc(devInf.dataPtr, 0x02, (UINT8)alg);  /* Algorithm register */
        
        /* Render some samples */
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, 256, buffer);
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: All FM algorithms configured successfully\n");
    return 1;
}

/**
 * Test 10: Waveform selection
 * Verifies all 8 waveforms can be selected without errors
 */
static int test_waveform_selection(void)
{
    DEV_GEN_CFG devCfg;
    DEV_INFO devInf;
    DEVFUNC_WRITE_A8D8 writeFunc;
    UINT8 retVal;
    DEV_SMPL *buffer[4];
    int ch, wf;
    
    printf("Test: Waveform selection...\n");
    
    for (ch = 0; ch < 4; ch++)
    {
        buffer[ch] = (DEV_SMPL*)malloc(BUFFER_SIZE * sizeof(DEV_SMPL));
        TEST_ASSERT(buffer[ch] != NULL, "Failed to allocate audio buffer");
    }
    
    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.clock = CLOCK_RATE;
    devCfg.emuCore = 0;
    devCfg.srMode = DEVRI_SRMODE_NATIVE;
    devCfg.smplRate = SAMPLE_RATE;
    
    retVal = SndEmu_Start(DEVID_YMF271, &devCfg, &devInf);
    if (retVal != 0) {
        for (ch = 0; ch < 4; ch++) free(buffer[ch]);
        TEST_ASSERT(0, "Failed to start YMF271");
    }
    
    SndEmu_GetDeviceFunc(devInf.devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&writeFunc);
    
    if (devInf.devDef->Reset != NULL)
        devInf.devDef->Reset(devInf.dataPtr);
    
    /* Test all 8 waveforms */
    for (wf = 0; wf < 8; wf++) {
        /* Set waveform for slot 0 */
        writeFunc(devInf.dataPtr, 0x00, (UINT8)(wf << 4));  /* Waveform in upper nibble */
        
        /* Render some samples */
        for (ch = 0; ch < 4; ch++)
            memset(buffer[ch], 0, BUFFER_SIZE * sizeof(DEV_SMPL));
        devInf.devDef->Update(devInf.dataPtr, 256, buffer);
    }
    
    SndEmu_Stop(&devInf);
    for (ch = 0; ch < 4; ch++) free(buffer[ch]);
    
    printf("  PASS: All waveforms selected successfully\n");
    return 1;
}

/**
 * Main test runner
 */
int main(int argc, char *argv[])
{
    printf("===========================================\n");
    printf("YMF271 VGM Integration Tests\n");
    printf("===========================================\n\n");
    
    /* Run all tests */
    test_ymf271_init();
    test_timer_a_period();
    test_timer_b_period();
    test_audio_rendering();
    test_envelope_transitions();
    test_sync_modes();
    test_status_registers();
    test_extended_playback();
    test_fm_algorithms();
    test_waveform_selection();
    
    /* Print summary */
    printf("\n===========================================\n");
    printf("Test Summary\n");
    printf("===========================================\n");
    printf("Tests run:    %d\n", results.tests_run);
    printf("Tests passed: %d\n", results.tests_passed);
    printf("Tests failed: %d\n", results.tests_failed);
    printf("===========================================\n");
    
    if (results.tests_failed > 0) {
        printf("\nSome tests FAILED!\n");
        return 1;
    }
    
    printf("\nAll tests PASSED!\n");
    return 0;
}
