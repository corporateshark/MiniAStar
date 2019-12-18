#include <assert.h>
#include <stdio.h>
#include <string>

#include "MiniAStar.h"

int main()
{
	constexpr unsigned kDim = 20;
	std::string maze[kDim] =
	{
		"++++++++++++++++++++",
		"+                  +",
		"++++++++++++++     +",
		"++++++++++++++     +",
		"++++++++++++++++++ +",
		"++++++++++++++++++ +",
		"+++  ++       ++++ +",
		"+++  ++   +++  +++ +",
		"+++       +++   ++ +",
		"+++   +++++++   ++ +",
		"+++++ +++++++      +",
		"+++++ ++++++++++++++",
		"+++++ +      +++++++",
		"+++++   ++         +",
		"+++++++++++++++++  +",
		"+++++++++++++++++  +",
		"++++++++           +",
		"+++++++            +",
		"+        +++++++   +",
		"++++++++++++++++++++",
	};

	assert(maze[0].length() == kDim);

	std::vector<miniastar::Point> outPath;

	auto passabilityFunc = [maze](unsigned x, unsigned y) -> bool { return maze[y][x] == ' '; };
	auto heuristicFunc = [](auto x1, auto y1, auto x2, auto y2) -> int { return miniastar::heuristicManhattan(x1, y1, x2, y2); };
	auto weightFunc = [](unsigned curx, unsigned cury, unsigned newx, unsigned newy) -> int { return 0; };

	const int length = miniastar::AStarGrid(
		{ 1,1 }, { 1,18 },
		kDim, kDim,
		&outPath,
		passabilityFunc,
		heuristicFunc,
		weightFunc,
		miniastar::eConnectivity::eConnectivity_4
	);

	for (const auto& p : outPath)
		maze[p.y][p.x] = '@';

	for (unsigned y = 0; y != kDim; y++)
	{
		for (unsigned x = 0; x != kDim; x++)
			printf("%c", maze[y][x]);
		printf("\n");
	}

	return 0;
}