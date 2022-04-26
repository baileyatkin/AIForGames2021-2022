#include "AIManager.h"
#include "Vehicle.h"
#include "DrawableGameObject.h"
#include "PickupItem.h"
#include "Waypoint.h"
#include "main.h"
#include "constants.h"
#include "MyStructures.h"

AIManager::AIManager()
{
	m_bCar = nullptr;
    m_rCar = nullptr;
    wandering = false;
    seeking = false;
    fleeing = false;
    pathing = false;
    rwp = nullptr;
}

AIManager::~AIManager()
{
	release();
}

void AIManager::release()
{
	clearDrawList();

	for (PickupItem* pu : m_pickups)
	{
		delete pu;
	}
	m_pickups.clear();

	delete m_bCar;
	m_bCar = nullptr;

    delete m_rCar;
    m_rCar = nullptr;

    delete rwp;
    rwp = nullptr;
}

HRESULT AIManager::initialise(ID3D11Device* pd3dDevice)
{
    // create the vehicle 
    float xPos = -500; // an abtrirary start point
    float yPos = 300;

    m_bCar = new Vehicle();
    HRESULT hr = m_bCar->initMesh(pd3dDevice, carColour::blueCar);
    m_bCar->setVehiclePosition(Vector2D(xPos, yPos));
    if (FAILED(hr))
        return hr;

    m_rCar = new Vehicle();
    HRESULT hr_2 = m_rCar->initMesh(pd3dDevice, carColour::redCar);
    m_rCar->setVehiclePosition(Vector2D(xPos, yPos));
    if (FAILED(hr_2))
        return hr_2;

    // setup the waypoints
    m_waypointManager.createWaypoints(pd3dDevice);
    m_bCar->setWaypointManager(&m_waypointManager);
    m_rCar->setWaypointManager(&m_waypointManager);

    // create a passenger pickup item
    PickupItem* pPickupPassenger = new PickupItem();
    hr = pPickupPassenger->initMesh(pd3dDevice, pickuptype::Passenger);
    m_pickups.push_back(pPickupPassenger);

    // NOTE!! for fuel and speedboost - you will need to create these here yourself

    // (needs to be done after waypoint setup)
    setRandomPickupPosition(pPickupPassenger);

    for (unsigned int i = 0; i < m_waypointManager.getWaypointCount(); i++) {
        node* tempNode = new node();
        tempNode->position = m_waypointManager.getWaypoint(i)->getPosition();
        nodes.push_back(tempNode);
    }

    return hr;
}


void AIManager::update(const float fDeltaTime)
{
    for (unsigned int i = 0; i < m_waypointManager.getWaypointCount(); i++) {
        m_waypointManager.getWaypoint(i)->update(fDeltaTime);
        AddItemToDrawList(m_waypointManager.getWaypoint(i)); // if you uncomment this, it will display the waypoints
    }

    //for (int i = 0; i < m_waypointManager.getQuadpointCount(); i++)
    //{
    //    Waypoint* qp = m_waypointManager.getQuadpoint(i);
    //    qp->update(fDeltaTime);
    //    AddItemToDrawList(qp); // if you uncomment this, it will display the quad waypoints
    //}

    // update and display the pickups
    for (unsigned int i = 0; i < m_pickups.size(); i++) {
        m_pickups[i]->update(fDeltaTime);
        AddItemToDrawList(m_pickups[i]);
    }

    // draw the waypoints nearest to the car
    Waypoint* wp = m_waypointManager.getNearestWaypoint(m_bCar->getPosition());
    if (wp != nullptr)
    {
        vecWaypoints vwps = m_waypointManager.getNeighbouringWaypoints(wp);
        for (Waypoint* wp : vwps)
        {
            AddItemToDrawList(wp);
        }
    }

    if (wandering)
    {
        if ((time(0) - timer > 2) || (m_rCar->getPosition() == targetPos))
        {
            timer = time(0);
            rwp = m_waypointManager.getRandomWaypoint();
            targetPos = rwp->getPosition();
            m_rCar->setPositionTo(rwp->getPosition());
        }
    }

    if (seeking)
    {
        m_bCar->setPositionTo(m_rCar->getPosition());
    }

    if (fleeing)
    {
        //Program will check if the red car is close enough to the blue car, if it is, the program will gather the neighbouring waypoints to the waypoint the blue car is currently on, comparing them all to
        //see which is the furthest away from the red car, the blue car will then travel to that waypoint
        if (fabs(bluePos.x - redPos.x) <= 50.0 && fabs(bluePos.y - redPos.y) <= 50.0)
        {
            vecWaypoints nwps = m_waypointManager.getNeighbouringWaypoints(m_waypointManager.getNearestWaypoint(bluePos));
            Vector2D tempPos = bluePos;
            for (Waypoint* twp : nwps)
            {
                Vector2D wpPos = twp->getPosition();
                Vector2D pos1;
                Vector2D pos2;
                pos1.x = fabs(redPos.x - tempPos.x);
                pos1.y = fabs(redPos.y - tempPos.y);
                pos2.x = fabs(redPos.x - wpPos.x);
                pos2.y = fabs(redPos.y - wpPos.y);
                if (fabs((pos1.x * pos1.x) - (pos1.y - pos1.y)) < fabs((pos2.x * pos2.x) - (pos2.y * pos2.y)))
                {
                    tempPos = wpPos;
                }
            }

            m_bCar->setPositionTo(tempPos);
            OutputDebugStringA("Red car still near \n");
        }
    }

    if (pathing)
    {
        if (!nodePath.empty())
        {
            m_bCar->setPositionTo(nodePath.front()->position);
            if (bluePos == nodePath.front()->position)
            {
                nodePath.pop();
            }
        }
        else
        {
            pathing = false;
        }
    }

    // update and draw the cars (and check for pickup collisions)
	if (m_bCar != nullptr)
	{
		m_bCar->update(fDeltaTime);
		checkForCollisions();
		AddItemToDrawList(m_bCar);
        bluePos = m_bCar->getPosition();
	}

    if (m_rCar != nullptr)
    {
        m_rCar->update(fDeltaTime);
        checkForCollisions();
        AddItemToDrawList(m_rCar);
        redPos = m_rCar->getPosition();
    }
}

void AIManager::mouseUp(int x, int y)
{
	// get a waypoint near the mouse click, then set the car to move to the this waypoint
	Waypoint* wp = m_waypointManager.getNearestWaypoint(Vector2D(x, y));
	if (wp == nullptr)
		return;

    node* carNode = new node();
    node* goalNode = new node();
    carNode->position = bluePos;
    goalNode->position = wp->getPosition();
    pathfinding(carNode, goalNode);

    // steering mode
    m_bCar->setPositionTo(nodePath.front()->position);
    pathing = true;
}

void AIManager::keyUp(WPARAM param)
{
    const WPARAM key_a = 65;
    switch (param)
    {
        case key_a:
        {
            OutputDebugStringA("a Up \n");
            break;
        }
    }
}

void AIManager::pathfinding(node* startNode, node* endNode)
{

    auto heuristic = [](node* nodeA, node* nodeB)
    {
        double x = abs(nodeA->position.x - nodeB->position.x);
        double y = abs(nodeA->position.y - nodeB->position.y);
        return x + y;
    };

    node* currentNode = new node();
    typedef pair<node*, float> costNode;
    priority_queue<costNode, vector<costNode>, greater<costNode>> pathQueue;
    pathQueue.emplace(make_pair(startNode, 0.0f));
    unordered_map<node*, node*> cameFrom;
    unordered_map<node*, double> costSoFar;
    costSoFar[startNode] = 0;
    cameFrom[startNode] = startNode;

    while (!pathQueue.empty())
    {
        currentNode = pathQueue.top().first;
        pathQueue.pop();

        if (currentNode->position == endNode->position)
            break;

        currentNode->neighbours = getNodeNeighbours(currentNode);

        for (auto neighbour : currentNode->neighbours)
        {
            double newCost = costSoFar[currentNode] + heuristic(endNode, neighbour);

            if (costSoFar.find(neighbour) == costSoFar.end() || newCost < costSoFar[neighbour])
            {
                costSoFar[neighbour] = newCost;
                cameFrom[neighbour] = currentNode;
                pathQueue.emplace(neighbour, newCost);
            }
        }
    }
    vector<node*> optimalPath;
    node* current = endNode;
    while (current != startNode)
    {
        optimalPath.push_back(current);
        current = cameFrom[current];
    }
    optimalPath.push_back(startNode);
    std::reverse(optimalPath.begin(), optimalPath.end());
    for (node* nextNode : optimalPath)
    {
        nodePath.push(nextNode);
    }
}

vecNodes AIManager::getNodeNeighbours(node* currentNode)
{
    vecNodes nodeVector;
    vecWaypoints wpNeighbours = m_waypointManager.getNeighbouringWaypoints(m_waypointManager.getNearestWaypoint(currentNode->position));
    for (Waypoint* wp : wpNeighbours)
    {
        for (node* tNode : nodes)
        {
            if (tNode->position == wp->getPosition())
            {

                nodeVector.push_back(tNode);
            }
        }
    }
    return nodeVector;
}

void AIManager::keyDown(WPARAM param)
{
	// hint 65-90 are a-z
	const WPARAM key_a = 65;
	const WPARAM key_s = 83;
    const WPARAM key_t = 84;
    const WPARAM key_w = 87;
    const WPARAM key_p = 80;
    const WPARAM key_f = 70;
    const WPARAM key_space = 32;

    switch (param)
    {
        case VK_NUMPAD0:
        {
            OutputDebugStringA("0 pressed \n");
            break;
        }
        case VK_NUMPAD1:
        {
            OutputDebugStringA("1 pressed \n");
            break;
        }
        case VK_NUMPAD2:
        {
            OutputDebugStringA("2 pressed \n");
            break;
        }
        case key_a:
        {
            OutputDebugStringA("a Down \n");

            break;
        }
        case key_w:
        {
            if (!wandering)
            {
                timer = time(0);
                Waypoint* rwp = m_waypointManager.getRandomWaypoint();
                targetPos = rwp->getPosition();
                m_rCar->setPositionTo(rwp->getPosition());
                wandering = true;
                break;
            }
            if (wandering)
            {
                wandering = false;
                break;
            }
        }
        case key_space:
        {
            mouseUp(0, 0);
            break;
        }

		case key_s:
		{
            m_rCar->setPositionTo(m_waypointManager.getRandomWaypoint()->getPosition());
			break;
		}
        case key_t:
		{
            break;
        }

        case key_p:
        {
            if (!seeking)
            {
                seeking = true;
                m_bCar->setPositionTo(m_rCar->getPosition());
                break;
            }
            if (seeking)
            {
                seeking = false;
                break;
            }
            break;
        }

        case key_f:
        {
            if (!fleeing)
            {
                fleeing = true;
                break;
            }
            if (fleeing)
            {
                fleeing = false;
                break;
            }
        }
        // etc
        default:
            break;
    }
}

void AIManager::setRandomPickupPosition(PickupItem* pickup)
{
    if (pickup == nullptr)
        return;
    int x = (rand() % SCREEN_WIDTH) - (SCREEN_WIDTH / 2);
    int y = (rand() % SCREEN_HEIGHT) - (SCREEN_HEIGHT / 2);

    Waypoint* wp = m_waypointManager.getNearestWaypoint(Vector2D(x, y));
    if (wp) {
        pickup->setPosition(wp->getPosition());
    }
}

/*
// IMPORTANT
// hello. This is hopefully the only time you are exposed to directx code 
// you shouldn't need to edit this, but marked in the code below is a section where you may need to add code to handle pickup collisions (speed boost / fuel)
// the code below does the following:
// gets the *first* pickup item "m_pickups[0]"
// does a collision test with it and the car
// creates a new random pickup position for that pickup

// the relevant #includes are already in place, but if you create your own collision class (or use this code anywhere else) 
// make sure you have the following:
#include <d3d11_1.h> // this has the appropriate directx structures / objects
#include <DirectXCollision.h> // this is the dx collision class helper
using namespace DirectX; // this means you don't need to put DirectX:: in front of objects like XMVECTOR and so on. 
*/

bool AIManager::checkForCollisions()
{
    if (m_pickups.size() == 0)
        return false;

    XMVECTOR dummy;

    // get the position and scale of the car and store in dx friendly xmvectors
    XMVECTOR carPos;
    XMVECTOR carScale;
    XMMatrixDecompose(
        &carScale,
        &dummy,
        &carPos,
        XMLoadFloat4x4(m_bCar->getTransform())
    );

    // create a bounding sphere for the car
    XMFLOAT3 scale;
    XMStoreFloat3(&scale, carScale);
    BoundingSphere boundingSphereCar;
    XMStoreFloat3(&boundingSphereCar.Center, carPos);
    boundingSphereCar.Radius = scale.x;

    // do the same for a pickup item
    // a pickup - !! NOTE it is only referring the first one in the list !!
    // to get the passenger, fuel or speedboost specifically you will need to iterate the pickups and test their type (getType()) - see the pickup class
    XMVECTOR puPos;
    XMVECTOR puScale;
    XMMatrixDecompose(
        &puScale,
        &dummy,
        &puPos,
        XMLoadFloat4x4(m_pickups[0]->getTransform())
    );

    // bounding sphere for pickup item
    XMStoreFloat3(&scale, puScale);
    BoundingSphere boundingSpherePU;
    XMStoreFloat3(&boundingSpherePU.Center, puPos);
    boundingSpherePU.Radius = scale.x;

	// THIS IS generally where you enter code to test each type of pickup
    // does the car bounding sphere collide with the pickup bounding sphere?
    if (boundingSphereCar.Intersects(boundingSpherePU))
    {
        OutputDebugStringA("Pickup Collision\n");
        m_pickups[0]->hasCollided();
        setRandomPickupPosition(m_pickups[0]);

        // you will need to test the type of the pickup to decide on the behaviour
        // m_pCar->dosomething(); ...

        return true;
    }

    return false;
}





