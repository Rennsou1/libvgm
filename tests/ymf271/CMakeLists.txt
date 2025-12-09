# YMF271 Emulation Tests
# 
# This CMakeLists.txt defines the test executables for YMF271 sound chip emulation.
# Tests are built when BUILD_TESTS is enabled in the main CMakeLists.txt.
#
# Test files:
#   - test_pfm.c: PFM (PCM-based FM) mode property tests
#   - test_timer_b.c: Timer B period calculation property tests
#   - test_vgm_integration.c: VGM file integration tests
#
# Usage:
#   cmake -DBUILD_TESTS=ON ..
#   cmake --build .
#   ./bin/ymf271_pfm_test
#   ./bin/ymf271_timer_b_test
#   ./bin/ymf271_vgm_integration_test

# PFM Mode Tests
# Tests Property 11: PFM Flag Storage
# Tests Property 12: PFM Mode Carrier Selection
# Validates: Requirements 7.1, 7.2, 7.3
add_executable(ymf271_pfm_test test_pfm.c)
target_include_directories(ymf271_pfm_test PRIVATE ${LIBVGM_SOURCE_DIR})
target_link_libraries(ymf271_pfm_test PRIVATE vgm-emu)
if(USE_SANITIZERS)
    add_sanitizers(ymf271_pfm_test)
endif(USE_SANITIZERS)

# Timer B Tests
# Tests Property 2: Timer Period Calculation
# Tests Property 3: Timer Reset Behavior
# Validates: Requirements 2.1, 2.2, 2.3, 2.4
add_executable(ymf271_timer_b_test test_timer_b.c)
target_include_directories(ymf271_timer_b_test PRIVATE ${LIBVGM_SOURCE_DIR})
target_link_libraries(ymf271_timer_b_test PRIVATE vgm-emu)
if(USE_SANITIZERS)
    add_sanitizers(ymf271_timer_b_test)
endif(USE_SANITIZERS)

# VGM Integration Tests
# Tests YMF271 emulation with real VGM files from Raiden Fighters
# Validates: All Requirements
add_executable(ymf271_vgm_integration_test test_vgm_integration.c)
target_include_directories(ymf271_vgm_integration_test PRIVATE ${LIBVGM_SOURCE_DIR})
target_link_libraries(ymf271_vgm_integration_test PRIVATE vgm-emu)
if(USE_SANITIZERS)
    add_sanitizers(ymf271_vgm_integration_test)
endif(USE_SANITIZERS)

# Install test executables
install(TARGETS ymf271_pfm_test ymf271_timer_b_test ymf271_vgm_integration_test
        DESTINATION "${CMAKE_INSTALL_BINDIR}")
