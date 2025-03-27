#ifndef _FIFO_BUFFER_H_
#define _FIFO_BUFFER_H_

#include <array.h>

template<class DType, unsigned int size>
class FifoBuffer
{
    public:
        void init(DType value = 0)
        {
            for (unsigned int i = 0; i < size; i++)
            {
                this->x[i] = value;
            }
        }   

        void add(DType x)   
        {
            for (int i = (int)(size-1); i > 0; i--)
            {
                this->x[i] = this->x[i-1];
            }

            this->x[0] = x;
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
        Array<DType, size> x;
};

#endif