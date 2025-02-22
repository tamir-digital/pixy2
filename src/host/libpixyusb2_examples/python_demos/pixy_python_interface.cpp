#include <string.h>
#include "libpixyusb2.h"
#include <stdio.h>

Pixy2  pixy_instance;

// Define frame dimensions constants
const int RAW_FRAME_WIDTH = 316;
const int RAW_FRAME_HEIGHT = 208;

// Debug control
static bool debug_enabled = false;

// Declare as pure C function to prevent name mangling
#ifdef __cplusplus
extern "C" {
#endif

void set_debug(bool enable)
{
    debug_enabled = enable;
}

#ifdef __cplusplus
}
#endif

int init()
{
  if (debug_enabled) printf("Debug: Attempting to initialize Pixy2...\n");
  int result = pixy_instance.init();
  if (debug_enabled) printf("Debug: Pixy2.init() returned %d\n", result);
  return result;
}

int stop()
{
  if (debug_enabled) printf("Debug: Stopping Pixy2...\n");
  int result = pixy_instance.m_link.stop();
  if (debug_enabled) printf("Debug: Pixy2.stop() returned %d\n", result);
  return result;
}

int resume()
{
  if (debug_enabled) printf("Debug: Resuming Pixy2...\n");
  int result = pixy_instance.m_link.resume();
  if (debug_enabled) printf("Debug: Pixy2.resume() returned %d\n", result);
  return result;
}

int get_raw_frame(uint8_t *frame)
{
  if (debug_enabled) printf("Debug: Getting raw frame...\n");
  uint8_t *bayerFrame;
  int result = pixy_instance.m_link.getRawFrame(&bayerFrame);
  if (debug_enabled) printf("Debug: getRawFrame() returned %d\n", result);
  if (result < 0) return result;
  
  // Copy the frame data
  memcpy(frame, bayerFrame, RAW_FRAME_WIDTH * RAW_FRAME_HEIGHT);
  return 0;
}

int change_prog (const char *  program_name)
{
  return pixy_instance.changeProg (program_name);
}

int get_frame_width ()
{
  return pixy_instance.frameWidth;
}

int get_frame_height ()
{
  return pixy_instance.frameHeight;
}

void video_get_RGB (int  X, int  Y, uint8_t *  Red, uint8_t *  Green, uint8_t *  Blue)
{
  pixy_instance.video.getRGB (X, Y, Red, Green, Blue);
}

int ccc_get_blocks (int  max_blocks, struct Block *  blocks)
{
  int  number_of_blocks_copied;
  int  blocks_available;
  int  index;
  
  // Get number of blocks detected //
  pixy_instance.ccc.getBlocks();
  blocks_available = pixy_instance.ccc.numBlocks;

  // Copy the maximum amount of blocks that can be copied //
  number_of_blocks_copied = (max_blocks >= blocks_available ? blocks_available : max_blocks);   

  // Copy blocks //
  for (index = 0; index != number_of_blocks_copied; ++index)  {
    memcpy (&blocks[index], &pixy_instance.ccc.blocks[index], sizeof (Block));
  }

  return number_of_blocks_copied;
}

void line_get_all_features ()
{
  pixy_instance.line.getAllFeatures();
}

void line_get_main_features ()
{
  pixy_instance.line.getMainFeatures();
}

int line_get_vectors (int  max_vectors, struct Vector *  vectors)
{
  int  number_of_vectors_copied;
  int  vectors_available;
  int  index;

  vectors_available        = pixy_instance.line.numVectors;
  number_of_vectors_copied = (max_vectors >= vectors_available ? vectors_available : max_vectors);

  // Copy vectors //
  for (index = 0; index != number_of_vectors_copied; ++index)  {
    memcpy (&vectors[index], &pixy_instance.line.vectors[index], sizeof (Vector));
  }

  return number_of_vectors_copied;
}

int line_get_intersections (int  max_intersections, struct Intersection *  intersections)
{
  int  number_of_intersections_copied;
  int  intersections_available;
  int  index;

  intersections_available        = pixy_instance.line.numIntersections;
  number_of_intersections_copied = (max_intersections >= intersections_available ? intersections_available : max_intersections);

  // Copy intersections //
  for (index = 0; index != number_of_intersections_copied; ++index)  {
    memcpy (&intersections[index], &pixy_instance.line.intersections[index], sizeof (Intersection));
  }

  return number_of_intersections_copied;
}

int line_get_barcodes (int  max_barcodes, struct Barcode *  barcodes)
{
  int  number_of_barcodes_copied;
  int  barcodes_available;
  int  index;

  barcodes_available        = pixy_instance.line.numBarcodes;
  number_of_barcodes_copied = (max_barcodes >= barcodes_available ? barcodes_available : max_barcodes);

  // Copy barcodes //
  for (index = 0; index != number_of_barcodes_copied; ++index) {
    memcpy (&barcodes[index], &pixy_instance.line.barcodes[index], sizeof (Barcode));
  }

  return number_of_barcodes_copied;
}

void set_lamp (int upper, int lower)
{
  pixy_instance.setLamp (upper, lower);
}

void set_servos (int  S1_Position, int  S2_Position)
{
  pixy_instance.setServos (S1_Position, S2_Position);
}

// Add getter functions for frame dimensions
int get_raw_frame_width()
{
    return RAW_FRAME_WIDTH;
}

int get_raw_frame_height()
{
    return RAW_FRAME_HEIGHT;
}
