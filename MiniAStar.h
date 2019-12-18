/**
 * \file MiniAStar.h
 * \brief
 *
 * Minimalistic header-only A* pathfinding algorithm implementation in C++14
 *
 * \version 1.0.0
 * \date 18/12/2019
 * \author Sergey Kosarevsky, 2019
 * \author sk@linderdaum.com   http://www.linderdaum.com   http://blog.linderdaum.com
 * \copyright MIT License
 */

 /*
	 Usage example:

		 Check MiniAStar.cpp
 */


 /* Versions history:
  *		1.0.0     Dec 18, 2019
 */

#pragma once

#include <limits>
#include <queue>
#include <vector>

namespace miniastar
{
	enum class eConnectivity : uint8_t
	{
		eConnectivity_4,
		eConnectivity_8,
	};

	struct Point
	{
		int x, y;

		bool operator == (const Point& other) const { return other.x == x && other.y == y; }
		bool operator != (const Point& other) const { return !(*this == other); }
	};

	struct QueueNode
	{
		Point pt;
		int dist;

		bool operator < (const QueueNode& other) const { return dist > other.dist; }
	};

	int heuristicManhattan(int x1, int y1, int x2, int y2) { return abs(x1 - x2) + abs(y1 - y2); }
	int heuristicDiagonal(int x1, int y1, int x2, int y2) { return std::max(abs(x1 - x2), abs(y1 - y2)); }
	int heuristicEuclidean(int x1, int y1, int x2, int y2) { return static_cast<int>(sqrt(double((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)))); }

	template <
		typename passabilityFunc_t,
		typename heuristicFunc_t,
		typename weightFunc_t
	>
		int AStarGrid(
			const Point& src, const Point& dst,
			unsigned gridDimX, unsigned gridDimY,
			std::vector<Point>* outPath,
			passabilityFunc_t passabilityFunc,
			heuristicFunc_t heuristicFunc,
			weightFunc_t weightFunc,
			eConnectivity connectivity = eConnectivity::eConnectivity_8
		)
	{
		std::priority_queue<QueueNode> q;
		q.push({ src, 0 });

		std::vector<Point> prev(gridDimX * gridDimY); // for backtracking
		std::vector<int> cost(gridDimX * gridDimY);

		std::fill(cost.begin(), cost.end(), std::numeric_limits<int>::max());
		cost[src.y * gridDimX + src.x] = 0;
		prev[src.y * gridDimX + src.x] = src;

		constexpr int headingX[] = { -1,  0,  0, +1, +1, -1, +1, -1 };
		constexpr int headingY[] = {  0, -1, +1,  0, +1, -1, -1, +1 };

		const int numHeadings = connectivity == eConnectivity::eConnectivity_4 ? 4 : 8;

		while (!q.empty())
		{
			const QueueNode curr = q.top();

			if (curr.pt == dst)
			{
				// backtracking to reconstruct the path
				if (outPath)
				{
					Point pt = curr.pt;
					while (prev[pt.y * gridDimX + pt.x] != src)
					{
						outPath->push_back(pt);
						pt = prev[pt.y * gridDimX + pt.x];
					}
					outPath->push_back(pt);
				}
				return curr.dist;
			}

			q.pop();

			const int currIdx = curr.pt.y * gridDimX + curr.pt.x;

			for (int i = 0; i < numHeadings; i++)
			{
				const int x = curr.pt.x + headingX[i];
				const int y = curr.pt.y + headingY[i];
				const int newIdx = y * gridDimX + x;

				if (!passabilityFunc(x, y)) continue;

				const int weight = 1 + weightFunc(curr.pt.x, curr.pt.y, x, y);
				const int newCost = cost[currIdx] + weight;
				if (newCost < cost[newIdx])
				{
					q.push({ { x, y }, newCost + heuristicFunc(dst.x, dst.y, x, y) });
					cost[newIdx] = newCost;
					prev[newIdx] = curr.pt;
				}
			}
		}

		return -1;
	}
} // namespace miniastar
