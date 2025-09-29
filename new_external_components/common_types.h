//
//  common_types.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-28.
//

using MilliSec = unsigned long;

constexpr float CtoF(float celsius) {
    return (celsius * 1.8f) + 32.0f;
}

constexpr float FtoC(float fahrenheit) {
    return (fahrenheit - 32.0f) / 1.8f;
}
