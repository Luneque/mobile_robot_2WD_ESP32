// Stub minimo do Arduino.h para compilar os headers do firmware no PC.
//
// Existe apenas para os testes nativos (tests/native): os headers do
// firmware incluem <Arduino.h> e usam, na pratica, pouquissimo da API —
// hoje so PI e abs(). Se um dia precisarem de mais, o stub cresce aqui
// e o CI aponta o erro.
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define PROGMEM
#define F(x) x

// Serial simulado: os headers do firmware usam Serial.print/println, e o
// teste do gravador exercita dumpSample()/dump(). Basta engolir a saida.
struct StubSerial {
    void begin(unsigned long) {}
    template <typename T> void print(const T&) {}
    template <typename T> void print(const T&, int) {}
    template <typename T> void println(const T&) {}
    template <typename T> void println(const T&, int) {}
    void println() {}
};
static StubSerial Serial;

// Arduino define abs() como macro (serve para int e float).
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
