#ifndef TARGET_DETECTION_H
#define TARGET_DETECTION_H

#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "dl_lib.h"

// Target detection parameters
#define TARGET_COLOR_R 255  // Red component of target color
#define TARGET_COLOR_G 0    // Green component of target color
#define TARGET_COLOR_B 0    // Blue component of target color
#define COLOR_THRESHOLD 50  // Color difference threshold

// Function to detect a target in an image
bool detectTarget(uint8_t* jpgBuffer, size_t jpgLength, int* targetX, int* targetY, float* confidence) {
  // Convert JPEG to RGB format
  uint8_t* rgbBuffer = NULL;
  size_t rgbWidth = 0;
  size_t rgbHeight = 0;
  
  bool converted = jpg2rgb(jpgBuffer, jpgLength, &rgbBuffer, JPG_SCALE_NONE);
  if (!converted) {
    Serial.println("JPEG conversion failed");
    return false;
  }
  
  // Get frame dimensions
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    free(rgbBuffer);
    Serial.println("Failed to get camera frame buffer");
    return false;
  }
  
  rgbWidth = fb->width;
  rgbHeight = fb->height;
  esp_camera_fb_return(fb);
  
  // Simple color-based target detection
  // This is a basic implementation - you might want to use more sophisticated
  // computer vision techniques for better detection
  
  int totalPixels = rgbWidth * rgbHeight;
  int targetPixelCount = 0;
  long sumX = 0;
  long sumY = 0;
  
  // Scan through the image looking for pixels matching target color
  for (int y = 0; y < rgbHeight; y++) {
    for (int x = 0; x < rgbWidth; x++) {
      int idx = (y * rgbWidth + x) * 3; // RGB has 3 bytes per pixel
      
      // Get RGB values
      uint8_t r = rgbBuffer[idx];
      uint8_t g = rgbBuffer[idx + 1];
      uint8_t b = rgbBuffer[idx + 2];
      
      // Calculate color difference
      int rDiff = abs(r - TARGET_COLOR_R);
      int gDiff = abs(g - TARGET_COLOR_G);
      int bDiff = abs(b - TARGET_COLOR_B);
      
      // Check if pixel color is close to target color
      if (rDiff + gDiff + bDiff < COLOR_THRESHOLD) {
        // This pixel is part of the target
        targetPixelCount++;
        sumX += x;
        sumY += y;
      }
    }
  }
  
  // Free the RGB buffer
  free(rgbBuffer);
  
  // Calculate target position if enough matching pixels found
  if (targetPixelCount > 10) {
    *targetX = sumX / targetPixelCount;
    *targetY = sumY / targetPixelCount;
    *confidence = (float)targetPixelCount / (totalPixels / 100); // Percentage of matching pixels
    
    Serial.printf("Target detected at X=%d, Y=%d with confidence %.2f%%\n", 
                  *targetX, *targetY, *confidence);
    return true;
  }
  
  return false;
}

#endif // TARGET_DETECTION_H
