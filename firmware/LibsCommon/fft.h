#ifndef _FFT_H_
#define _FFT_H_

#include <stdint.h>
#include <Array.h>

constexpr double PI = 3.14159265358979323846;

//fixed point format 16.16
constexpr int32_t FIXED_POINT_SCALE = (uint32_t)(1 << 16);

typedef int32_t fixed_point_t;


// Convert a fixed-point number to floating-point
constexpr float fixed_to_float(fixed_point_t num) 
{
    return static_cast<float>(num) / FIXED_POINT_SCALE;
}

// Multiply two fixed-point numbers
constexpr fixed_point_t fixed_mul(fixed_point_t a, fixed_point_t b) 
{
    return static_cast<fixed_point_t>(((int64_t)a * b) >> 16);
}

// Structure to represent a fixed-point complex number
struct Complex 
{
    fixed_point_t real;
    fixed_point_t imag;

    Complex operator+(const Complex& other) const 
    {
        return {real + other.real, imag + other.imag};
    }

    Complex operator-(const Complex& other) const 
    {
        return {real - other.real, imag - other.imag};
    }

    Complex operator*(const Complex& other) const 
    {
        return {
            fixed_mul(real, other.real) - fixed_mul(imag, other.imag),
            fixed_mul(real, other.imag) + fixed_mul(imag, other.real)
        };
    }
};

// Precomputed sine and cosine tables
template<uint32_t N>
struct SinCosTable 
{
    Array<Complex, N / 2> table; 

    constexpr SinCosTable() : table{} 
    {
        for (uint32_t k = 0; k < N / 2; ++k) 
        {
            float angle = -2 * PI * k / N;
            table[k] = {float_to_fixed(cos(angle)), float_to_fixed(sin(angle))};
        }
    }
};

// Recursive FFT function template
template<uint32_t N>
void fft(Array<Complex, N>& x_in_out, const Array<Complex, N / 2>& table) 
{
    if constexpr (N <= 1) 
    {
        return; 
    } 
    else 
    {
        Array<Complex, N/2> x_even;
        Array<Complex, N/2> x_odd;

        for (uint32_t i = 0; i < N/2; ++i) 
        {
            x_even[i] = x_in_out[i * 2];
            x_odd[i]  = x_in_out[i * 2 + 1];
        }

        fft(x_even, table);
        fft(x_odd, table);

        for (uint32_t k = 0; k < N/2; ++k) 
        {
            Complex t = table[k] * x_odd[k];
            x_in_out[k] = x_even[k] + t;
            x_in_out[k + N/2] = x_even[k] - t;
        }
    }
}

#endif