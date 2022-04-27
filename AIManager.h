#pragma once

#include "WaypointManager.h"
#include "MyStructs.h"

using namespace std;

class Vehicle;
class PickupItem;
typedef vector<PickupItem*> vecPickups;
typedef vector<node*> vecNodes;

class AIManager
{
public:
	AIManager();
	virtual  ~AIManager();
	void	release();
	HRESULT initialise(ID3D11Device* pd3dDevice);
	void	update(const float fDeltaTime);
	void	mouseUp(int x, int y);
	void	keyDown(WPARAM param);
	void	keyUp(WPARAM param);
	void	pathfinding(Waypoint* startNode, Waypoint* endNode, Vehicle* car);
	vecNodes getNodeNeighbours(node* currentNode);
	void	clearPath(queue<Waypoint*>);

protected:
	bool	checkForCollisions();
	void	setRandomPickupPosition(PickupItem* pickup);

private:
	vecPickups              m_pickups;
	Vehicle*				m_bCar = nullptr;
	Vehicle*				m_rCar = nullptr;
	WaypointManager			m_waypointManager;
	bool					wandering;
	bool					seeking;
	bool					fleeing;
	bool					pathing;
	time_t					timer;
	Waypoint*				rwp;
	Vector2D				targetPos;
	Vector2D				redPos;
	Vector2D				bluePos;
	queue<Waypoint*>			bluePath;
	queue<Waypoint*>			redPath;
	node*					redNode;
	node*					blueNode;
	vector<node*>			nodes;


};

