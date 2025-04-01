#ifndef _LINE_MAPPING_H_
#define _LINE_MAPPING_H_

#include <array.h>

#define LINE_MAPPING_D_MAX  ((float)1000000000)

template<unsigned int size, unsigned int seg_count>
class LineMapping
{
    public:
        LineMapping()
        {
            init();
        }

        void init()
        {
            this->distance_prev = 0.0;
            this->x = 0.0;
            this->y = 0.0;
            
            float distance_map = 0.0;

            s_pos.set(-LINE_MAPPING_D_MAX);
            s_len.set(0);
        }

        void step(float distance, float angle)
        {   
            float dif = distance - distance_prev;
            distance_prev = distance;

            float dx = dif * np.cos(angle);
            float dy = dif * np.sin(angle);

            this->x+= dx;
            this->y+= dy;
        }

        public:
            int find_closest_straight_segment()
            {
                int     idx = -1;
                float d_min = LINE_MAPPING_D_MAX;

                for (unsigned int i = 0; i < seg_count; i++)
                {
                    float d = seg_distance(i);
                    if (d < d_min)
                    {
                        d_min = d;
                        idx   = i;
                    }   
                }
                    
                return idx;
            }


            float seg_distance(unsigned int idx)
            {
                return  abs(s_pos[idx] - distance_map);
            }


    private:
        float distance_prev;
        float x, y;
        Array<float, size> x, y;

    private:
        float distance_map;
        // straight segment mapping, 
        // distance from beigning position
        // and length of segment
        Array<float,   seg_count> s_pos;
        Array<float,   seg_count> s_len;
};

#endif