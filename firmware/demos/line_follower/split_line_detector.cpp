#include "split_line_detector.h"

#include "fmath.h"
#include "drivers.h"


void SplitLineDetector::init(float max_distance, float threshold)
{
  this->max_distance = max_distance;
  this->threshold    = threshold;

  state           = 0;
  distance_mark   = 0;
  left_position   = 0;
  right_position  = 0;
}

void SplitLineDetector::reset()
{
  state       = 0;
}

int SplitLineDetector::step(float distance, float left_position, float right_position)
{
  if (state == 0) 
  {
    // right side steep curve
    if (right_position > threshold)
    {
      distance_mark        = distance + max_distance;
      this->left_position  = left_position;
      this->right_position = right_position;

      state                = 1;
    }

    // left side steep curve
    if (left_position < -threshold)
    {
      distance_mark        = distance + max_distance;
      this->left_position  = left_position;
      this->right_position = right_position;

      state                = 2;
    }
  }

  else if (state == 1)
  {
    // too far distance, reset into initial state
    if (distance > distance_mark)
    {
      state = 0;
    }
    else  
    {
      // right position suddenly changed to left
      if (left_position < -threshold)
      {
        distance_mark  = distance + max_distance;

        // move to waiting state
        state = 3;
        
        return -1;
      }
    }
  }


  else if (state == 2)
  {
    // too far distance, reset into initial state
    if (distance > distance_mark)
    {
      state = 0;
    }
    else  
    {
      // left position suddenly changed to right
      if (right_position > threshold)
      {
        distance_mark  = distance + max_distance;

        // move to waiting state
        state = 3;
        
        return 1;
      }
    }
  }



  else if (state == 3)
  {
    if (distance > distance_mark)
    {
      state = 0;
    }
  }

  return 0;
}

