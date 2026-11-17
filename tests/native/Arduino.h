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

// Arduino define abs() como macro (serve para int e float).
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
