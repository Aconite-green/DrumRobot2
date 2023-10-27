#pragma once
#ifndef CONNECT
#define CONNECT
#include <vector>
using namespace std;

class connect
{
	vector<double> Q1;
	vector<double> Q2;
	int k;
	int n;
	vector<double> Qi;

public:
	connect(vector<double>& q1, vector<double>& q2, int k_val, int n_val);
	vector<double> Run();
};


#endif // !CONNECT