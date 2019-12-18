# MiniAStar
Minimalistic header-only A* pathfinding algorithm implementation in C++14
--------

# Easy to use:

```C++
const char maze[kDim][kDim] = ...;

std::vector<miniastar::Point> outPath;

auto passabilityFunc = [maze](unsigned x, unsigned y) -> bool
   { return maze[x][y] == ' '; };
auto heuristicFunc = [](auto x1, auto y1, auto x2, auto y2) -> int
   { return miniastar::heuristicManhattan(x1, y1, x2, y2); };
auto weightFunc = [](unsigned curx, unsigned cury, unsigned newx, unsigned newy) -> int
   { return 0; };

const int length = miniastar::AStarGrid(
	{ 1,1 }, { 1,18 },
	kDim, kDim,
	&outPath,
	passabilityFunc,
	heuristicFunc,
	weightFunc,
	miniastar::eConnectivity::eConnectivity_4
);
```

--------
8-connected and 4-connected paths.
![Demo Animation](https://github.com/corporateshark/MiniAStar/blob/media/screenshot.png?raw=true)
