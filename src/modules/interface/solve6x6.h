#pragma once

//#include <stdio.h>
//#include <stdlib.h>
#include <math.h> // fabs()

#define N (6)

void swap_row(float (*a)[N], float b[N], int r1, int r2)
{
	float tmp;
	int i;

	if (r1 == r2) return;

	for (i = 0; i < N; i++) {
		tmp = a[r1][i];
		a[r1][i] = a[r2][i];
		a[r2][i] = tmp;
	}

	tmp = b[r1];
	b[r1] = b[r2];
	b[r2] = tmp;
}

// THIS IS A DESTRUCTIVE, IN-PLACE FUNCTION
// a and b will be destroyed!!
void solve6x6(float a[N][N], float b[N], float x[N])
{
#define A(y, x) (a[y][x])
	int j, col, row, max_row,dia;
	float max, tmp;

	for (dia = 0; dia < N; dia++) {
		max_row = dia, max = A(dia, dia);

		for (row = dia + 1; row < N; row++)
			if ((tmp = fabs(A(row, dia))) > max)
				max_row = row, max = tmp;

		swap_row(a, b, dia, max_row);

		for (row = dia + 1; row < N; row++) {
			tmp = A(row, dia) / A(dia, dia);
			for (col = dia+1; col < N; col++)
				A(row, col) -= tmp * A(dia, col);
			A(row, dia) = 0;
			b[row] -= tmp * b[dia];
		}
	}
	for (row = N - 1; row >= 0; row--) {
		tmp = b[row];
		for (j = N - 1; j > row; j--)
			tmp -= x[j] * A(row, j);
		x[row] = tmp / A(row, row);
	}
#undef A
}

#undef N
