/*
 * Copyright 2021 Infinitap Games
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _SPRING_H_
#define _SPRING_H_

// Assume DT = 1. The DT needs to be premulitplied to the spring factor and the exponent needs to be
// calculated accordingly
inline void Spring_Premult_Update( float *outPos, float *outVel, float pos, float vel, float springFactor_dt, float springExp_dt )
{
	const float B         = vel + springFactor_dt * pos;
	const float P1       = ( pos + B ) * springExp_dt;
	const float V1       = B * springExp_dt - springFactor_dt * P1;
	*outPos = P1;
	*outVel = V1;
}

inline void Spring_Premult_Lerp( float *outPos, float *outVel, float pos, float vel, float targetPos, float springFactor_dt, float springExp_dt )
{
	const float diff = pos - targetPos;
	float diffDt;
	Spring_Premult_Update( &diffDt, outVel, diff, vel, springFactor_dt, springExp_dt );
	*outPos = targetPos + diffDt;
}

inline float Spring_Exp( float springFactor_dt )
{
	return expf( -springFactor_dt );	
}

inline void Spring_Update( float *outPos, float *outVel, float pos, float vel, float springFactor, float dt )
{
	// p(dt) = (pos + ( vel + k * pos ) * dt ) * e^(-k*dt) 
	// B = ( vel + k * pos )
	// p(dt) = (pos + B * dt ) * e^(-k*dt) 
	// v(dt) = B * e^(-k*dt) + -k * e^(-k*dt) * (pos + B * dt )  
	// v(dt) = e^(-k*dt) * ( B - k * (pos + B * dt ) )
	const float springExp = Spring_Exp( springFactor * dt );
	const float B         = vel + springFactor * pos;
	const float Pdt       = ( pos + B * dt ) * springExp;
	const float Vdt       = B * springExp - springFactor * Pdt;
	*outPos = Pdt;
	*outVel = Vdt;
}

inline void Spring_Lerp( float *outPos, float *outVel, float pos, float vel, float targetPos, float springFactor, float dt )
{
	const float diff = pos - targetPos;
	float diffDt;
	Spring_Update( &diffDt, outVel, diff, vel, springFactor, dt );
	*outPos = targetPos + diffDt;
}

inline void Spring_LerpAngleDegree(float *outPos, float *outVel, float pos, float vel, float targetPos, float springFactor, float dt)
{
	const float diff = float_AngleModDeg(pos - targetPos);
	float diffDt;
	Spring_Update(&diffDt, outVel, diff, vel, springFactor, dt);
	*outPos = float_AngleModDeg(targetPos + diffDt);
}

template<typename T>
void Spring_VecUpdate( T *outPos, T *outVel, const T &pos, const T &vel, float springFactor, float dt )
{
	// p(dt) = (pos + ( vel + k * pos ) * dt ) * e^(-k*dt) 
	// B = ( vel + k * pos )
	// p(dt) = (pos + B * dt ) * e^(-k*dt) 
	// v(dt) = B * e^(-k*dt) + -k * e^(-k*dt) * (pos + B * dt )  
	// v(dt) = e^(-k*dt) * ( B - k * (pos + B * dt ) )
	const float springExp = Spring_Exp( springFactor * dt );
	const T   B         = vel + springFactor * pos;
	const T   Pdt       = ( pos + B * dt ) * springExp;
	const T   Vdt       = B * springExp - springFactor * Pdt;
	*outPos = Pdt;
	*outVel = Vdt;
}

template<typename T>
void Spring_VecLerp( T *outPos, T *outVel, const T &pos, const T &vel, const T &targetPos, float springFactor, float dt )
{
	const T diff = pos - targetPos;
	T diffDt;
	Spring_VecUpdate( &diffDt, outVel, diff, vel, springFactor, dt );
	*outPos = targetPos + diffDt;
}

template<typename T>
class SpringVec
{
public:
	void Reset()
	{
		this->pos = T();
		this->vel = T();
		this->target = T();
	}

	void Update( float springFactor, float dt )
	{
		Spring_VecLerp( &this->pos, &this->vel, this->pos, this->vel, this->target, springFactor, dt );
	}

	const T &GetTarget() const
	{
		return this->target;
	}

	const T &GetPos() const 
	{
		return this->pos;
	}

	const T &GetVel() const 
	{
		return this->vel;
	}

	void ForcePos( const T &pos )
	{
		this->pos = pos;
		this->vel = T();
		this->target = pos;
	}

	void SetTarget( const T &target )
	{
		this->target = target;
	}

	T pos;
	T vel;
	T target;
};

class Spring
{
public:
	void Reset()
	{
		this->pos = 0.0f;
		this->vel = 0.0f;
		this->target = 0.0f;
	}

	void SetTarget( float target )
	{
		this->target = target;
	}

	void Update( float springFactor, float dt )
	{
		Spring_Lerp( &this->pos, &this->vel, this->pos, this->vel, this->target, springFactor, dt );
	}

	float GetTarget() const
	{
		return this->target;
	}

	float GetPos() const 
	{
		return this->pos;
	}

	float GetVel() const 
	{
		return this->vel;
	}

	void ForcePos( float pos )
	{
		this->pos = pos;
		this->vel = 0.0f;
		this->target = pos;
	}

	float pos;
	float vel;
	float target;
};

// MRG: this is in degrees! 
class SpringAngle
{
public:
	void Reset()
	{
		this->pos = 0.0f;
		this->vel = 0.0f;
		this->target = 0.0f;
	}

	void SetTarget(float target)
	{
		this->target = target;
	}

	void Update(float springFactor, float dt)
	{
		Spring_LerpAngleDegree(&this->pos, &this->vel, this->pos, this->vel, this->target, springFactor, dt);
	}

	float GetTarget() const
	{
		return this->target;
	}

	float GetPos() const
	{
		return this->pos;
	}

	float GetVel() const
	{
		return this->vel;
	}

	void ForcePos(float pos)
	{
		this->pos = pos;
		this->vel = 0.0f;
		this->target = pos;
	}

	float pos;
	float vel;
	float target;
};

class Spring_Premult
{
public:
	void Reset()
	{
		this->pos = 0.0f;
		this->vel = 0.0f;
		this->target = 0.0f;
	}

	void SetTarget( float target )
	{
		this->target = target;
	}

	void Update( float springFactor, float springExp )
	{
		// precaclulated, but to generate: springExp = Spring_Exp( springFactor );
		Spring_LerpAngleDegree(&this->pos, &this->vel, this->pos, this->vel, this->target, springFactor, springExp);
	}

	float GetTarget() const
	{
		return this->target;
	}

	float GetPos() const 
	{
		return this->pos;
	}

	float GetVel() const 
	{
		return this->vel;
	}

	float pos;
	float vel;
	float target;
};

#if ASSERTS
#define Spring_Premult_Verify( factor, exp ) ASSERT( float_ApproxEquals( Spring_Exp( factor ), exp ) )
#else // #if ASSERTS
#define Spring_Premult_Verify( factor, exp ) ((void)0)
#endif // #else // #if ASSERTS


#endif // #ifndef _SPRING_H_
