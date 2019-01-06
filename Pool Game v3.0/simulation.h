/*-----------------------------------------------------------
  Simulation Header File
  -----------------------------------------------------------*/
#include"vecmath.h"

/*-----------------------------------------------------------
  Macros
  -----------------------------------------------------------*/
#define TABLE_X			(0.6f) 
#define TABLE_Z			(1.2f)
#define TABLE_Y			(0.1f)
#define BALL_RADIUS		(0.05f)
#define BALL_MASS		(0.1f)
#define TWO_PI			(6.2832f)
#define	SIM_UPDATE_MS	(10)
#define NUM_BALLS		(1)		
#define NUM_CUSHIONS	(44) 		
#define MAX_PARTICLES	(200)
#define NUM_HOLES		(4)
/*-----------------------------------------------------------
  plane normals
  -----------------------------------------------------------*/
/*
extern vec2	gPlaneNormal_Left;
extern vec2	gPlaneNormal_Top;
extern vec2	gPlaneNormal_Right;
extern vec2	gPlaneNormal_Bottom;
*/


/*-----------------------------------------------------------
  cushion class
  -----------------------------------------------------------*/
class cushion
{
public:
	vec2	vertices[2]; //2d
	vec2	centre;
	vec2	normal;

	void MakeNormal(void);
	void MakeCentre(void);
};

/*-----------------------------------------------------------
  ball class
  -----------------------------------------------------------*/

class ball
{
	static int ballIndexCnt;
public:
	vec2	position;
	vec2	velocity;
	float	radius;
	float	mass;
	int		index;

	ball(): position(0.0), velocity(0.0), radius(BALL_RADIUS), 
		mass(BALL_MASS) {index = ballIndexCnt++; Reset();}
	
	void Reset(void);
	void ApplyImpulse(vec2 imp);
	void ApplyFrictionForce(int ms);
	void DoPlaneCollision(const cushion &c);
	void DoBallCollision(ball &b);
	void Update(int ms);
	
	bool HasHitPlane(const cushion &c) const;
	bool HasHitBall(const ball &b) const;
	bool HasHitHole();

	void HitPlane(const cushion &c);
	void HitBall(ball &b);
};

class particle 
{
public:
	vec3 position;
	vec3 velocity;

	particle() {position=0;velocity=0;}
	void update(int ms);
};

class particleSet 
{
public:
	particle *particles[MAX_PARTICLES];
	int num;

	particleSet()
	{
		for(int i=0;i<MAX_PARTICLES;i++) particles[i] = 0;
		num=0;
	}

	~particleSet()
	{
		for(int i=0;i<MAX_PARTICLES;i++)
		{
			if(particles[i]) delete particles[i];
		}
	}

	void AddParticle(const vec3 &pos);
	void update(int ms);
};

class hole
{
public:
	vec2 position;

};

/*-----------------------------------------------------------
  table class
  -----------------------------------------------------------*/
class table
{
public:
	hole holes[4];
	ball balls[NUM_BALLS];
	cushion cushions[NUM_CUSHIONS];
	particleSet parts;

	void SetupCushions(void);
	void SetupBall(void);
	void Update(int ms);
	bool AnyBallsMoving(void) const;

	

private:

	

	float course1[48]
	{ 
		0, -1, 1, -1,
		1, -1, 1, -2,
		1, -2, 2, -2,
		2, -2, 2, -3,
		2, -3, 1, -3,
		1, -3, 1, -4,
		1, -4, 0, -4,
		0, -4, 0, -3,
		0, -3, -1, -3,
		-1, -3, -1, -2,
		-1, -2, 0, -2,
		0, -2, 0, -1
	};

	float course2[40]
	{ 
		4, -1, 5, -1,
		5, -1, 5, -3,
		5, -3, 4, -3,
		4, -3, 4, -4,
		4, -4, 5, -4,
		5, -4, 5, -5,
		5, -5, 3, -5,
		3, -5, 3, -2,
		3, -2, 4, -2,
		4, -2, 4, -1
	};

	float course3[48]
	{ 
		6, -2, 7, -2,
		7, -2, 7, -1,
		7, -1, 10, -1,
		10, -1, 10, -4,
		10, -4, 7, -4,
		7, -4, 7, -3,
		7, -3, 6, -3,
		6, -3, 6, -2,
		//interior
		8, -2, 8, -3,
		8, -3, 9, -3,
		9, -3, 9, -2,
		9, -2, 8, -2
	};

	float course4[40]
	{
		11, -2, 12, -2,
		12, -2, 12, -1,
		12, -1, 15, -1,
		15, -1, 15, -3,
		15, -3, 14, -3,
		14, -3, 14, -2,
		14, -2, 13, -2,
		13, -2, 13, -4,
		13, -4, 11, -4,
		11, -4, 11, -2
	};
};
/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;