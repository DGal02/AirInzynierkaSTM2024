#pragma once

struct EncoderParams
{
	double imp2rad = 5.98364147543706e-9;
	double rad2imp = 1 / imp2rad;
	double imp2arc = 0.0012;
	double arc2imp = 1 / imp2arc;
	double imp2m = 1.0e-9;
	double mechanicalOffset = 0.0;
};
