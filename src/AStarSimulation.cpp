// Milan Fatschel
// Path Finder Simulation
// INSTRUCTIONS for GUI
// KEY:
// green = start, red = end, black = obstacle, white = open tiles, grey = closed tiles
// left mouse click = toggle obstacle/open tile

#include <iostream>
#include <algorithm>
#include <queue>

#include "olcConsoleGameEngine.h"

class PathFinder : public olcConsoleGameEngine
{
public:
	PathFinder()
	{
		m_sAppName = L"Path Finder";
	}
private:
	struct Node {
		bool m_bObstacle = false;
		bool m_bVisited = false;
		float m_globalGoal, m_localGoal;
		int m_x, m_y;
		Node* m_parent;
		std::vector<Node*> m_nodeNeighbors;

		// Formula calculating distance between this node and another node
		float getDistanceToNode(Node* otherNode) {
			return sqrtf((m_x - otherNode->m_x) * (m_x - otherNode->m_x) + (m_y - otherNode->m_y) * (m_y - otherNode->m_y));
		}
	};

	Node* m_nodes;
	int m_nMapWidth = 15;
	int m_nMapHeight = 15;

	Node* m_nodeStart = nullptr;
	Node* m_nodeEnd = nullptr;

	// Compare struct for priority queue
	struct compare {
		bool operator()(const Node* a, const Node* b) {
			return a->m_globalGoal > b->m_globalGoal;
		}
	};

protected:
	bool OnUserCreate()
	{
		// Create 2D array of Nodes in the form of an 1D array
		// (easier to keep track of x and y positions)
		m_nodes = new Node[m_nMapWidth * m_nMapHeight];

		// Init the positions and status of each node
		for(int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{
				m_nodes[y * m_nMapWidth + x].m_x = x;
				m_nodes[y * m_nMapWidth + x].m_y = y;
				m_nodes[y * m_nMapWidth + x].m_bObstacle = false;
				m_nodes[y * m_nMapWidth + x].m_parent = nullptr;
				m_nodes[y * m_nMapWidth + x].m_bVisited = false;
			}
		}

		// Create node connections 
		for (int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{
				if (x > 0)
					m_nodes[y * m_nMapWidth + x].m_nodeNeighbors.push_back(&m_nodes[(y + 0) * m_nMapWidth + (x - 1)]);
				if (y > 0)
					m_nodes[y * m_nMapWidth + x].m_nodeNeighbors.push_back(&m_nodes[(y - 1) * m_nMapWidth + (x + 0)]);
				if (x < m_nMapWidth - 1)
					m_nodes[y * m_nMapWidth + x].m_nodeNeighbors.push_back(&m_nodes[(y + 0) * m_nMapWidth + (x + 1)]);
				if (y < m_nMapHeight - 1)
					m_nodes[y * m_nMapWidth + x].m_nodeNeighbors.push_back(&m_nodes[(y + 1) * m_nMapWidth + (x + 0)]);
			}
		}

		// Initialize start and end nodes on first and last node
		m_nodeStart = &m_nodes[0];
		m_nodeEnd = &m_nodes[m_nMapWidth * m_nMapHeight - 1];
		return true;
	}

	bool OnUserUpdate(float fElapsedTime)
	{
		// Continue to run till application dies. Updates values continuously

		// Create box sizes and borders around them
		int nNodeSize = 5;
		float nNodeBorder = .1;

		// Get the mouses (input) location on the screen
		int nSelectedNodeX = m_mousePosX / nNodeSize;
		int nSelectedNodeY = m_mousePosY / nNodeSize;

		// On mouse click, find which node is closest to curser in the given grid.
		// Toggle obstacle on/off then run the A* search algorithm 
		if (m_mouse[0].bReleased)
		{
			if (nSelectedNodeX >= 0 && nSelectedNodeX < m_nMapWidth)
			{
				if (nSelectedNodeY >= 0 && nSelectedNodeY < m_nMapHeight)
				{
					m_nodes[nSelectedNodeY * m_nMapWidth + nSelectedNodeX].m_bObstacle = !m_nodes[nSelectedNodeY * m_nMapWidth + nSelectedNodeX].m_bObstacle;
					AStar();
					// dijkstra();
					// bfs();
				}
			}
		}

		// Draw Screen
		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		// Draw different colors to represent status:
		for (int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{

				Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
					(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
					PIXEL_HALF, m_nodes[y * m_nMapWidth + x].m_bObstacle ? FG_BLACK : FG_WHITE);

				if (m_nodes[y * m_nMapWidth + x].m_bVisited)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
					(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_SOLID, FG_GREY);

				if (&m_nodes[y * m_nMapWidth + x] == m_nodeStart)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder, 
					(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_SOLID, FG_BLUE);

				if (&m_nodes[y * m_nMapWidth + x] == m_nodeEnd)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
					(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_SOLID, FG_RED);
			}


			// Draw shortest path by A* in yellow if there is a path
			if (m_nodeEnd != nullptr)
			{
				Node* scan = m_nodeEnd;
				if(m_nodeEnd->m_parent != nullptr)
				    scan = m_nodeEnd->m_parent;
				while (scan->m_parent != nullptr)
				{
					Fill(scan->m_x * nNodeSize + nNodeBorder, scan->m_y * nNodeSize + nNodeBorder,
						(scan->m_x + 1) * nNodeSize - nNodeBorder, (scan->m_y + 1) * nNodeSize - nNodeBorder, PIXEL_HALF, FG_GREEN);
					scan = scan->m_parent;
				}
			}

		}

		return true;
	}

	void AStar()
	{
		// Function will find the shortest path from the start and end goal.
		// Will also take into account any obstacles put in place.

		// Reset all nodes to find the next shortest path
		for (int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{
				m_nodes[y * m_nMapWidth + x].m_bVisited = false;
				m_nodes[y * m_nMapWidth + x].m_globalGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_localGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_parent = nullptr;
			}
		}

		// Init the start node with a local value of zero. Find the distance
		// to end goal and update it to the global value
		Node* currentNode = m_nodeStart;
		m_nodeStart->m_localGoal = 0.0f;
		m_nodeStart->m_globalGoal = m_nodeStart->getDistanceToNode(m_nodeEnd);

		// Create a priority queue to keep the order of the least global
		// value to be searched first
		std::priority_queue<Node*, std::vector<Node*>, compare> nodeTestList;
		nodeTestList.push(m_nodeStart);

		while (!nodeTestList.empty())
		{
			// While the list is not empty and the top has been visited or it is the end node
			// remove it from the list. It does not need to be checked
			while (!nodeTestList.empty() && (nodeTestList.top()->m_bVisited || nodeTestList.top() == m_nodeEnd))
				nodeTestList.pop();
			if (nodeTestList.empty()) break; // if empty after popping, break out 

			// Start from highest priority
			currentNode = nodeTestList.top();
			currentNode->m_bVisited = true;

			// Check each neighbor of the current node
			for (auto nodeNeighbor : currentNode->m_nodeNeighbors)
			{
				// Calculate possible lower local goal
				float possibleLowerLocalGoal = currentNode->m_localGoal +
					currentNode->getDistanceToNode(nodeNeighbor);

				// If it is lower, update the current local goal
				// and the parent. Recalculate the new global goal
				// based on the update local goal
				if (possibleLowerLocalGoal < nodeNeighbor->m_localGoal)
				{
					nodeNeighbor->m_parent = currentNode;
					nodeNeighbor->m_localGoal = possibleLowerLocalGoal;
					nodeNeighbor->m_globalGoal = nodeNeighbor->m_localGoal +
						nodeNeighbor->getDistanceToNode(m_nodeEnd);
				}

				// If the node neighbor has not been visited and it is not
				// an obstacle - add it to the pqueue
				if (!nodeNeighbor->m_bVisited && !nodeNeighbor->m_bObstacle)
					nodeTestList.push(nodeNeighbor);
			}
		}
	}

	void dijkstra()
	{
		// Function will find the shortest path from the start and end goal.
		// Will also take into account any obstacles put in place.

		// Reset all nodes to find the next shortest path
		for (int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{
				m_nodes[y * m_nMapWidth + x].m_bVisited = false;
				m_nodes[y * m_nMapWidth + x].m_globalGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_localGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_parent = nullptr;
			}
		}

		// Init the start node with a local value of zero. Find the distance
		// to end goal and update it to the global value
		Node* currentNode = m_nodeStart;
		m_nodeStart->m_localGoal = 0.0f;

		// Create a queue to keep the order of the least global
		// value to be searched first
		std::queue<Node*> nodeTestList;
		nodeTestList.push(m_nodeStart);

		while (!nodeTestList.empty())
		{
			// While the list is not empty and the top has been visited or it is the end node
			// remove it from the list. It does not need to be checked
			while (!nodeTestList.empty() && (nodeTestList.front()->m_bVisited || nodeTestList.front() == m_nodeEnd))
				nodeTestList.pop();
			if (nodeTestList.empty()) break; // if empty after popping, break out 

			// Start from highest priority
			currentNode = nodeTestList.front();
			currentNode->m_bVisited = true;

			// Check each neighbor of the current node
			for (auto nodeNeighbor : currentNode->m_nodeNeighbors)
			{
				// Calculate possible lower local goal
				float possibleLowerLocalGoal = currentNode->m_localGoal +
					currentNode->getDistanceToNode(nodeNeighbor);

				// If it is lower, update the current local goal
				// and the parent. Recalculate the new global goal
				// based on the update local goal
				if (possibleLowerLocalGoal < nodeNeighbor->m_localGoal)
				{
					nodeNeighbor->m_parent = currentNode;
					nodeNeighbor->m_localGoal = possibleLowerLocalGoal;
				}

				// If the node neighbor has not been visited and it is not
				// an obstacle - add it to the pqueue
				if (!nodeNeighbor->m_bVisited && !nodeNeighbor->m_bObstacle)
					nodeTestList.push(nodeNeighbor);
			}
		}
	}

	void bfs()
	{
		// breadth-first search that will search all surrounding nodes to reach the end point

		// Reset all nodes to find the next shortest path
		for (int x = 0; x < m_nMapWidth; x++)
		{
			for (int y = 0; y < m_nMapHeight; y++)
			{
				m_nodes[y * m_nMapWidth + x].m_bVisited = false;
				m_nodes[y * m_nMapWidth + x].m_globalGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_localGoal = INFINITY;
				m_nodes[y * m_nMapWidth + x].m_parent = nullptr;
			}
		}

		// Init the start node with a local value of zero. Find the distance
		// to end goal and update it to the global value
		Node* currentNode = m_nodeStart;

		// Create a queue to keep track of the next node to visit
		std::queue<Node*> nodeTestList;
		nodeTestList.push(m_nodeStart);

		// While the queue is not empty test the nodes
		while (!nodeTestList.empty())
		{
			// While the list is not empty and the top has been visited or it is the end node
			// remove it from the list. It does not need to be checked
			while (!nodeTestList.empty() && (nodeTestList.front()->m_bVisited || nodeTestList.front() == m_nodeEnd))
				nodeTestList.pop();
			if (nodeTestList.empty()) break; // if empty after popping, break out 

			// Start at front node in queue
			currentNode = nodeTestList.front();
			currentNode->m_bVisited = true;

			// Check each neighbor of the current node
			for (auto nodeNeighbor : currentNode->m_nodeNeighbors)
			{
				// If the node neighbor has not been visited and it is not
				// an obstacle - add it to the queue

				if (!nodeNeighbor->m_bVisited && !nodeNeighbor->m_bObstacle)
				{
					nodeNeighbor->m_parent = currentNode;
					nodeTestList.push(nodeNeighbor);
					if (nodeNeighbor == m_nodeEnd) return;
				}
			}
		}


	}
};


int main()
{
	PathFinder game;
	game.ConstructConsole(75, 75, 6, 6);
	game.Start();
	return 0;
}


