#include <Filters.h>
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <deque>
#include <vector>


// Test 1: Basic Average Computation for a Single Channel
// For a window size of 3, with inputs 1, 2, 3, the outputs should be:
//  1st call: (1)/3,  2nd call: (1+2)/3, 3rd call: (1+2+3)/3.
TEST(MovingAverageFilterTest, BasicAverageSingleChannel) {
    int windowSize = 3;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    double input = 1.0;
    filter.filter(&input);
    EXPECT_NEAR(input, 1.0 / windowSize, 1e-6);
    
    input = 2.0;
    filter.filter(&input);
    EXPECT_NEAR(input, (1.0 + 2.0) / windowSize, 1e-6);
    
    input = 3.0;
    filter.filter(&input);
    EXPECT_NEAR(input, (1.0 + 2.0 + 3.0) / windowSize, 1e-6);
}

// Test 2: Window Size of 1 (Input Should Be Returned as Output)
// With window size 1, the moving average should always equal the input.
TEST(MovingAverageFilterTest, WindowSizeOneSingleChannel) {
    int windowSize = 1;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    double input = 5.0;
    filter.filter(&input);
    EXPECT_NEAR(input, 5.0, 1e-6);
    
    input = 7.0;
    filter.filter(&input);
    EXPECT_NEAR(input, 7.0, 1e-6);
}

// Test 3: Multi-Channel Filtering
// For two channels with window size 3, verify that each channel’s moving average
// is computed independently.
TEST(MovingAverageFilterTest, MultiChannelBasicAverage) {
    int windowSize = 3;
    int lengthArray = 2;  // two channels
    MovingAverageFilter filter(windowSize, lengthArray);
    
    double data[2];
    
    // First call: Channel values: 1 and 10
    data[0] = 1.0; data[1] = 10.0;
    filter.filter(data);
    EXPECT_NEAR(data[0], (1.0) / windowSize, 1e-6);
    EXPECT_NEAR(data[1], (10.0) / windowSize, 1e-6);
    
    // Second call: Channel values: 2 and 20
    data[0] = 2.0; data[1] = 20.0;
    filter.filter(data);
    EXPECT_NEAR(data[0], (1.0 + 2.0) / windowSize, 1e-6);
    EXPECT_NEAR(data[1], (10.0 + 20.0) / windowSize, 1e-6);
    
    // Third call: Channel values: 3 and 30
    data[0] = 3.0; data[1] = 30.0;
    filter.filter(data);
    EXPECT_NEAR(data[0], (1.0 + 2.0 + 3.0) / windowSize, 1e-6);
    EXPECT_NEAR(data[1], (10.0 + 20.0 + 30.0) / windowSize, 1e-6);
}

// Test 4: Circular Buffer Behavior (Single Channel)
// With window size 3, after four calls the oldest value is dropped.
TEST(MovingAverageFilterTest, CircularBufferSingleChannel) {
    int windowSize = 3;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    double input = 1.0;
    filter.filter(&input); // Window: [1,0,0] -> average = 1/3
    input = 2.0;
    filter.filter(&input); // Window: [1,2,0] -> average = (1+2)/3
    input = 3.0;
    filter.filter(&input); // Window: [1,2,3] -> average = (1+2+3)/3 = 2.0
    input = 4.0;
    filter.filter(&input); // Window rotates: oldest (1) replaced by 4; New window: [4,2,3] -> average = (4+2+3)/3 = 3.0
    EXPECT_NEAR(input, 3.0, 1e-6);
}

// Test 5: Constant Input Stabilization (Single Channel)
// Repeatedly input a constant value so that, after the filter is filled, the output stabilizes.
TEST(MovingAverageFilterTest, ConstantInputStabilization) {
    int windowSize = 5;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    const double constantValue = 7.0;
    double output = 0.0;
    
    // Feed the filter with constant input repeatedly.
    // In each call, we start with 'constantValue' as the input,
    // and the filter overwrites the output with the moving average.
    for (int i = 0; i < windowSize; i++) {
         output = constantValue;  // reset output to constant for each call
         filter.filter(&output);
         // Optionally, you can print the output for debugging:
         // std::cout << "Iteration " << i << ": " << output << std::endl;
    }
    
    // Once the window is fully populated with constantValue,
    // the moving average should be equal to constantValue.
    EXPECT_NEAR(output, constantValue, 1e-6);
}



// Advanced Test 2: Random Noise Convergence
// Feed zero-mean normally distributed noise and check that the output remains near zero.
TEST(MovingAverageFilterAdvanced, RandomNoiseConvergence) {
    int windowSize = 50;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    std::default_random_engine generator(42);
    std::normal_distribution<double> distribution(0.0, 1.0);
    double input = 0.0;
    for (int i = 0; i < 1000; ++i) {
        input = distribution(generator);
        filter.filter(&input);
    }
    EXPECT_NEAR(input, 0.0, 0.2);  // Allow a tolerance of 0.2 due to noise.
}

// Advanced Test 3: Step Response Analysis
// Fill the filter with zeros, then apply a step input and verify the immediate average.
TEST(MovingAverageFilterAdvanced, StepResponse) {
    int windowSize = 5;
    int lengthArray = 1;
    MovingAverageFilter filter(windowSize, lengthArray);
    
    double input = 0.0;
    // Fill with zeros.
    for (int i = 0; i < windowSize; ++i) {
        filter.filter(&input);
    }
    // Apply a step to 10.0.
    input = 10.0;
    filter.filter(&input);
    double expected = 10.0 / windowSize;
    EXPECT_NEAR(input, expected, 1e-6);
}

// Advanced Test 4: Multi-Channel Sine/Cosine Response
// Test the filter on two channels with sine and cosine inputs.
TEST(MovingAverageFilterAdvanced, SineCosineMultiChannel) {
    int windowSize = 10;
    int lengthArray = 2;  // Two channels.
    MovingAverageFilter filter(windowSize, lengthArray);
    
    const double pi = 3.141592653589793;
    const int iterations = 50;
    double data[2] = {0.0, 0.0};
    
    // Simulate sine on channel 1 and cosine on channel 2.
    for (int i = 0; i < iterations; ++i) {
        data[0] = sin(2 * pi * i / iterations);
        data[1] = cos(2 * pi * i / iterations);
        filter.filter(data);
    }
    // Verify outputs remain within [-1, 1].
    EXPECT_GE(data[0], -1.0);
    EXPECT_LE(data[0], 1.0);
    EXPECT_GE(data[1], -1.0);
    EXPECT_LE(data[1], 1.0);
}


// Advanced Heavy Workload Test for Multi-Channel Filtering
TEST(MovingAverageFilterAdvanced, HeavyWorkloadRandomMultiChannel) {
    const int windowSize = 50;      // Size of the moving window
    const int numChannels = 4;      // Number of channels
    const int iterations = 10000;   // Total iterations for heavy workload
    
    // Create the filter with given window size and number of channels
    MovingAverageFilter filter(windowSize, numChannels);
    
    // Create circular buffers (one per channel) initialized to 0.0
    std::vector<std::deque<double>> channelBuffers(numChannels, std::deque<double>(windowSize, 0.0));
    
    double input[numChannels];      // This array will be passed to the filter
    // We'll use it both to provide raw input and later to read the output.
    
    // Set up a random number generator for inputs (for reproducibility, fixed seed)
    std::default_random_engine generator(42);
    std::uniform_real_distribution<double> distribution(-100.0, 100.0);
    
    // Perform many iterations of random inputs.
    for (int iter = 0; iter < iterations; ++iter) {
        for (int ch = 0; ch < numChannels; ++ch) {
            double value = distribution(generator);
            input[ch] = value;
            
            // Update the circular buffer for channel ch:
            channelBuffers[ch].pop_front();
            channelBuffers[ch].push_back(value);
        }
        
        // Pass the inputs to the filter.
        // The filter overwrites input[] with the moving average.
        filter.filter(input);
    }
    
    // After the iterations, compute the expected moving average for each channel.
    for (int ch = 0; ch < numChannels; ++ch) {
        double sum = 0.0;
        for (double v : channelBuffers[ch]) {
            sum += v;
        }
        double expectedAverage = sum / windowSize;
        
        // The filter's output is stored in input[]. Compare it to the expected average.
        EXPECT_NEAR(input[ch], expectedAverage, 1e-6) << "Channel " << ch << " mismatch.";
    }
}


TEST(MovingAverageFilterAdvanced, EdgeCases) {
    // Use a small window and two channels to expose edge behaviors.
    int windowSize = 3;
    int lengthArray = 2;
    MovingAverageFilter filter(windowSize, lengthArray);
    double data[2];

    // --- Edge Case 1: All zeros ---
    data[0] = 0.0; data[1] = 0.0;
    filter.filter(data);
    // Expect average = 0/3 = 0 for both channels.
    EXPECT_NEAR(data[0], 0.0, 1e-6);
    EXPECT_NEAR(data[1], 0.0, 1e-6);

    // --- Edge Case 2: Constant nonzero input ---
    data[0] = 10.0; data[1] = -5.0;
    filter.filter(data);
    // For first channel: (0 + 10)/3, second: (0 - 5)/3.
    EXPECT_NEAR(data[0], (0.0 + 10.0) / 3.0, 1e-6);
    EXPECT_NEAR(data[1], (0.0 - 5.0) / 3.0, 1e-6);

    // --- Edge Case 3: Abrupt step change ---
    data[0] = 100.0; data[1] = -100.0;
    filter.filter(data);
    // Now the window for channel 0 contains: [0, 10, 100] => average = (0+10+100)/3.
    // For channel 1: [0, -5, -100] => average = (0-5-100)/3.
    EXPECT_NEAR(data[0], (0.0 + 10.0 + 100.0) / 3.0, 1e-6);
    EXPECT_NEAR(data[1], (0.0 - 5.0 - 100.0) / 3.0, 1e-6);

    // --- Edge Case 4: Circular buffer behavior after window is full ---
    // Feed one more input; this will cause the oldest value to be dropped.
    data[0] = -50.0; data[1] = 50.0;
    filter.filter(data);
    // For channel 0, the new window should be: [10, 100, -50] => average = (10 + 100 - 50) / 3.
    // For channel 1, the new window should be: [-5, -100, 50] => average = (-5 - 100 + 50) / 3.
    EXPECT_NEAR(data[0], (10.0 + 100.0 - 50.0) / 3.0, 1e-6);
    EXPECT_NEAR(data[1], (-5.0 - 100.0 + 50.0) / 3.0, 1e-6);
}