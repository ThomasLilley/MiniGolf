/*-----------------------------------------------------------
  Simulation Source File
  -----------------------------------------------------------*/
#include"stdafx.h"
#include"simulation.h"
#include<stdio.h>
#include<stdlib.h>
#include<iostream>

/*-----------------------------------------------------------
  macros
  -----------------------------------------------------------*/
#define SMALL_VELOCITY		(0.01f)

/*-----------------------------------------------------------
  globals
  -----------------------------------------------------------*/
/*
vec2	gPlaneNormal_Left(1.0,0.0);
vec2	gPlaneNormal_Top(0.0,1.0);
vec2	gPlaneNormal_Right(-1.0,0.0);
vec2	gPlaneNormal_Bottom(0.0,-1.0);
*/

table gTable;

static const float gRackPositionX[] = {0.0f,0.0f,(BALL_RADIUS*2.0f),(-BALL_RADIUS*2.0f),(BALL_RADIUS*4.0f)}; 
static const float gRackPositionZ[] = {0.5f,0.0f,(-BALL_RADIUS*3.0f),(-BALL_RADIUS*3.0f)}; 

float gCoeffRestitution = 0.5f;
float gCoeffFriction = 0.03f;
float gGravityAccn = 9.8f;


/*-----------------------------------------------------------
  cushion class members
  -----------------------------------------------------------*/
void cushion::MakeNormal(void)
{
	//can do this in 2d
	vec2 temp = vertices[1]-vertices[0];
	normal(0) = temp(1);
	normal(1) = -temp(0);
	normal.Normalise();
}

void cushion::MakeCentre(void)
{
	centre = vertices[0];
	centre += vertices[1];
	centre/=2.0;
}

/*-----------------------------------------------------------
  ball class members
  -----------------------------------------------------------*/
int ball::ballIndexCnt = 0;

void ball::Reset(void)
{

	//set velocity to zero
	velocity = 0.0;

	//work out rack position
	if(index==0)
	{
		position(1) = 0.5;
		position(0) = 0.0;
		return;
	}
	
	static const float sep = (BALL_RADIUS*3.0f);
	static const float rowSep = (BALL_RADIUS*2.5f);
	int row = 1;
	int rowIndex = index;
	
		rowIndex -= row;
		row++;

	position(1) =  -(rowSep * (row-1));
	position(0) = (((row-1)*sep)/2.0f) - (sep*(row-rowIndex));
	
}

void ball::ApplyImpulse(vec2 imp)
{
	velocity = imp;
}

void ball::ApplyFrictionForce(int ms)
{
	if(velocity.Magnitude()<=0.0) return;

	//accelaration is opposite to direction of motion
	vec2 accelaration = -velocity.Normalised();
	//friction force = constant * mg
	//F=Ma, so accelaration = force/mass = constant*g
	accelaration *= (gCoeffFriction * gGravityAccn);
	//integrate velocity : find change in velocity
	vec2 velocityChange = ((accelaration * ms)/1000.0f);
	//cap magnitude of change in velocity to remove integration errors
	if(velocityChange.Magnitude() > velocity.Magnitude()) velocity = 0.0;
	else velocity += velocityChange;
}

void ball::DoBallCollision(ball &b)
{
	if(HasHitBall(b)) HitBall(b);
}

void ball::DoPlaneCollision(const cushion &b)
{
	if(HasHitPlane(b)) HitPlane(b);
}

void ball::Update(int ms)
{
	//apply friction
	ApplyFrictionForce(ms);
	//integrate position
	position += ((velocity * ms)/1000.0f);
	//set small velocities to zero
	if(velocity.Magnitude()<SMALL_VELOCITY) velocity = 0.0;

}


bool ball::HasHitPlane(const cushion &c) const
{
	//if moving away from plane, cannot hit
	if (velocity.Dot(c.normal) >= 0.0) {
		return false;
	}
	//if in front of plane, then have not hit
	vec2 relPos = position - c.vertices[0];
	
	bool collisionDetected = false;
	double cPos = 0;
	double collisionPos = 0;
	// determines whether the plane is horizontal or vertical
	if (c.vertices[0](0) == c.vertices[1](0) && c.vertices[0](1) != c.vertices[1](1)) {
		// if the course wall is on the vertical plane then detect collisions	
		cPos = 0;
		cPos = c.centre(0);
		collisionPos = (position(0) - cPos);
		if (collisionPos < 0) {
			collisionPos *= -1;
		}
		//creates a buffer so that the ball does not pass the wall before colliding
		if (collisionPos < 0.05) {

			// if the 	
			if (c.vertices[0](1) - c.vertices[1](1) < 0) {
				if (position(1) > c.vertices[0](1) && position(1) < c.vertices[1](1)) {
					collisionDetected = true;
				}
			}
			else if (c.vertices[0](1) - c.vertices[1](1) > 0) {
				if (position(1) < c.vertices[0](1) && position(1) > c.vertices[1](1)) { 				
					collisionDetected = true;;
				}
			}
		}
		else {
			return false;
		}
	}
	else if (c.vertices[0](1) == c.vertices[1](1) && c.vertices[0](0) != c.vertices[1](0)) {
		// if the course wall is on the horizontal plane then detect collisons
		cPos = 0;
		cPos = c.centre(1);

		collisionPos = (position(1) - cPos);
		if (collisionPos < 0) {
			collisionPos *= -1;
		}

		
		//creates a buffer so that the ball does not pass the wall before colliding
		if (collisionPos < 0.05) {
			if (c.vertices[0](0) - c.vertices[1](0) < 0) {
				if (position(0) > c.vertices[0](0) && position(0) < c.vertices[1](0)) 
				{  	
					collisionDetected = true;;
				}
			}
			else if (c.vertices[0](0) - c.vertices[1](0) > 0) {
				if (position(0) < c.vertices[0](0) && position(0) > c.vertices[1](0)) 
				{ 
					collisionDetected = true;;
				}
			}
		}
		else {
			return false;
		}
	}

	if (collisionDetected == true) {
		return true;
	}
	else
	{
		return false;
	}
}

bool ball::HasHitBall(const ball &b) const
{
	//work out relative position of ball from other ball,
	//distance between balls
	//and relative velocity
	vec2 relPosn = position - b.position;
	float dist = (float) relPosn.Magnitude();
	vec2 relPosnNorm = relPosn.Normalised();
	vec2 relVelocity = velocity - b.velocity;

	//if moving apart, cannot have hit
	if(relVelocity.Dot(relPosnNorm) >= 0.0) return false;
	//if distnce is more than sum of radii, have not hit
	if(dist > (radius+b.radius)) return false;
	return true;
}

bool ball::HasHitHole(const ball & b)
{
	double holex[4] = { 1.5, 4.5, 9.5, 14.5 };
	double holez[4] = { -2.5, -4.5, -2.5, -2.5 };

	double startposx[4] = { 0.5, 4.5, 6.5, 11.5 };
	double startposz[4] = { -1.5, -1.5, -2.5, -3.5 };



	for (int i = 0; i < NUM_HOLES; i++) {
		if (position(0) > holex[gTable.currentHole] - 0.1 &&position(0) < holex[gTable.currentHole] + 0.1) {
			if (position(1) > holez[gTable.currentHole] - 0.1 &&position(1) < holez[gTable.currentHole] + 0.1) {
				//std::cout << "Current Player:" << gTable.currentPlayer << std::endl;
				//std::cout << "Has Hit Hole !" << std::endl;
				position(0) = startposx[gTable.currentHole];
				position(1) = startposz[gTable.currentHole];
				velocity(0) = velocity(1) = 0;
				//std::cout << gTable.players[gTable.currentPlayer].holeCompleted << std::endl;
				if (gTable.currentPlayer < NUM_PLAYERS-1) {
					gTable.currentPlayer++;
				}
				else
				{
					gTable.currentPlayer = 0;
					if (gTable.currentHole < NUM_HOLES-1) {


						gTable.currentHole++;
						position(0) = startposx[gTable.currentHole];
						position(1) = startposz[gTable.currentHole];
						velocity(0) = velocity(1) = 0;
					}
					else
					{
						std::cout << "END OF GAME" << std::endl;
						gTable.balls[0].position(0) = 0.5;
						gTable.balls[0].position(1) = -1.5;
						gTable.gameOver = true;
					}
				}
				//std::cout << "Current Player:" << gTable.currentPlayer << std::endl;
			}
		}	
	}
	return false;
}

void ball::HitPlane(const cushion &c)
{
	//reverse velocity component perpendicular to plane  
	//std::cout << c.vertices[0](0) << c.vertices[0](1) << std::endl;
	double comp = velocity.Dot(c.normal) * (1.0+gCoeffRestitution);
	vec2 delta = -(c.normal * comp);
	velocity += delta; 

	//make some particles
	int n = (rand()%4)+3;
	vec3 pos(position(0),radius/2.0,position(1));
	vec3 oset(c.normal(0),0.0,c.normal(1));
	pos+=(oset*radius);
	for(int i=0;i<n;i++)
	{
		gTable.parts.AddParticle(pos);
	}

/*
	//assume elastic collision
	//find plane normal
	vec2 planeNorm = gPlaneNormal_Left;
	//split velocity into 2 components:
	//find velocity component perpendicular to plane
	vec2 perp = planeNorm*(velocity.Dot(planeNorm));
	//find velocity component parallel to plane
	vec2 parallel = velocity - perp;
	//reverse perpendicular component
	//parallel component is unchanged
	velocity = parallel + (-perp)*gCoeffRestitution;
*/
}

void ball::HitBall(ball &b)
{
	//find direction from other ball to this ball
	vec2 relDir = (position - b.position).Normalised();

	//split velocities into 2 parts:  one component perpendicular, and one parallel to 
	//the collision plane, for both balls
	//(NB the collision plane is defined by the point of contact and the contact normal)
	float perpV = (float)velocity.Dot(relDir);
	float perpV2 = (float)b.velocity.Dot(relDir);
	vec2 parallelV = velocity-(relDir*perpV);
	vec2 parallelV2 = b.velocity-(relDir*perpV2);
	
	//Calculate new perpendicluar components:
	//v1 = (2*m2 / m1+m2)*u2 + ((m1 - m2)/(m1+m2))*u1;
	//v2 = (2*m1 / m1+m2)*u1 + ((m2 - m1)/(m1+m2))*u2;
	float sumMass = mass + b.mass;
	float perpVNew = (float)((perpV*(mass-b.mass))/sumMass) + (float)((perpV2*(2.0*b.mass))/sumMass);
	float perpVNew2 = (float)((perpV2*(b.mass-mass))/sumMass) + (float)((perpV*(2.0*mass))/sumMass);
	
	//find new velocities by adding unchanged parallel component to new perpendicluar component
	velocity = parallelV + (relDir*perpVNew);
	b.velocity = parallelV2 + (relDir*perpVNew2);


	//make some particles
	int n = (rand()%5)+5;
	vec3 pos(position(0),radius/2.0,position(1));
	vec3 oset(relDir(0),0.0,relDir(1));
	pos+=(oset*radius);
	for(int i=0;i<n;i++)
	{
		gTable.parts.AddParticle(pos);
	}
}

/*-----------------------------------------------------------
  particle class members
  -----------------------------------------------------------*/
void particle::update(int ms)
{
	position += (velocity*ms)/1000.0;
	velocity(1) -= (4.0*ms)/1000.0; //(9.8*ms)/1000.0;
}

/*-----------------------------------------------------------
  particle set class members
  -----------------------------------------------------------*/
void particleSet::AddParticle(const vec3 &pos)
{
	if(num >= MAX_PARTICLES) return;
	particles[num] = new particle;
	particles[num]->position = pos;

	particles[num]->velocity(0) = ((rand() % 200)-100)/200.0;
	particles[num]->velocity(2) = ((rand() % 200)-100)/200.0;
	particles[num]->velocity(1) = 2.0*((rand() % 100)/100.0);

	num++;
}

void particleSet::update(int ms)
{
	int i=0;
	while(i<num)
	{
		particles[i]->update(ms);
		if((particles[i]->position(1) < 0.0) && (particles[i]->velocity(1)<0.0))
		{
			delete particles[i];
			particles[i] = particles[num-1];
			num--;
		}
		else i++;
	}
}


/*-----------------------------------------------------------
  table class members
  -----------------------------------------------------------*/
void table::SetupCushions(void)
{


	int cushion_cnt = 0;
	gTable.balls[0].position(0) = 0.5;
	gTable.balls[0].position(1) = -1.5;
	for (int i = 0; i < 12; i++)//hole 1
	{
		cushions[i].vertices[0](0) = course1[cushion_cnt++] ;
		cushions[i].vertices[0](1) = course1[cushion_cnt++] ; 
		cushions[i].vertices[1](0) = course1[cushion_cnt++] ; 
		cushions[i].vertices[1](1) = course1[cushion_cnt++] ; 

	}

	cushion_cnt = 0;
	for (int i = 12; i < 22; i++)//hole 2
	{
		cushions[i].vertices[0](0) = course2[cushion_cnt++] ;
		cushions[i].vertices[0](1) = course2[cushion_cnt++] ;
		cushions[i].vertices[1](0) = course2[cushion_cnt++] ;
		cushions[i].vertices[1](1) = course2[cushion_cnt++] ;

	}

	cushion_cnt = 0;
	for (int i = 22; i < 34; i++)//hole 3
	{
		cushions[i].vertices[0](0) = course3[cushion_cnt++] ;
		cushions[i].vertices[0](1) = course3[cushion_cnt++] ;
		cushions[i].vertices[1](0) = course3[cushion_cnt++] ;
		cushions[i].vertices[1](1) = course3[cushion_cnt++] ;

	}

	cushion_cnt = 0;
	for (int i = 34; i < 44; i++)//hole 4
	{
		
		cushions[i].vertices[0](0) = course4[cushion_cnt++] ;
		cushions[i].vertices[0](1) = course4[cushion_cnt++] ;
		cushions[i].vertices[1](0) = course4[cushion_cnt++] ;
		cushions[i].vertices[1](1) = course4[cushion_cnt++] ;

	}

	for(int i=0;i<NUM_CUSHIONS;i++)
	{
		cushions[i].MakeCentre();
		cushions[i].MakeNormal();
	}

	float holex[4] = { 1.5, 4.5, 9.5, 14.5 };
	float holez[4] = { -2.5, -4.5, -2.5, -2.5 };

	for (int i = 0; i < NUM_HOLES; i++) {
		gTable.holes[i].position(0) = holex[i];
		gTable.holes[i].position(1) = holez[i];
		std::cout << gTable.holes[i].position(0) << " "<< gTable.holes[i].position(1) << " " << i << std::endl;
	}
	
	for (int i = 0; i < NUM_PLAYERS; i++) {
		player newPlayer;
		newPlayer.playerNo = i;
		gTable.players.push_back(newPlayer);
	}

}
 

void table::SetupBall(void)
{
	
}

void table::Update(int ms)
{
	//check for collisions for each ball
	for(int i=0;i<NUM_BALLS;i++) 
	{
		for(int j=0;j<NUM_CUSHIONS;j++)
		{
			balls[i].DoPlaneCollision(cushions[j]);
			balls[i].HasHitHole(balls[i]);
		}

		for(int j=(i+1);j<NUM_BALLS;j++) 
		{
			balls[i].DoBallCollision(balls[j]);
			
		}
	}
	
	//update all balls
	for(int i=0;i<NUM_BALLS;i++) balls[i].Update(ms);

	//update particles
	parts.update(ms);

	//make some new particles
	//vec3 pos(0.0,BALL_RADIUS,0.0);
	//parts.AddParticle(pos);
}

bool table::AnyBallsMoving(void) const
{
	//std::cout << "any balls moving?" << std::endl;
	//return true if any ball has a non-zero velocity
	for (int i = 0; i < NUM_BALLS; i++)
	{
		if (balls[i].velocity(0) != 0.0 && balls[i].velocity(1) != 0.0) {
			
			return true;
		}

	}

	return false;
	
}
