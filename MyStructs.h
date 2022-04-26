#pragma once
#include <list>
#include <vector>
#include "Vector2D.h"
#include "constants.h"
using namespace std;

struct node
{
	Vector2D position = Vector2D();
	vector<node*> neighbours;
};