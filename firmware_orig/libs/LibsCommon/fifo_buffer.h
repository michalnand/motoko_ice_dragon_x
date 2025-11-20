#ifndef _FIFO_BUFFER_H_
#define _FIFO_BUFFER_H_

#include <array.h>

template<class DType, unsigned int buffer_size>
class FifoBuffer
{
    public:
        void init(DType value = 0)
        {
            for (unsigned int i = 0; i < buffer_size; i++)
            {
                this->x[i] = value;
            }
        }   

        void add(DType x)   
        {
            for (int i = (int)(buffer_size-1); i > 0; i--)
            {
                this->x[i] = this->x[i-1];
            }

            this->x[0] = x;
        }

        unsigned int size()
        {
            return buffer_size;
        }

        DType& operator[](unsigned int index)
        {
            return x[index];
        }

        const DType& operator[](unsigned int index) const
        {
            return x[index];
        }

    private:
        Array<DType, buffer_size> x;
};

#endif