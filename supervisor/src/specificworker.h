/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 
	

private:
	enum class Estado {BUSCAR,IR,PARAR};
	Estado estado = Estado::BUSCAR;
	struct Tag{
		mutable QMutex m;
		int id;
		InnerModel *inner;
		QVec pose;
		
		void init(InnerModel *innev){
				inner = innev;
		}	

		void copy(int x, int z, int _id){
			QMutexLocker lm(&m);
			pose=inner->transform("world",QVec::vec3(x,0,z),"base"); 
			id = _id;
		}

};Tag tag;

};
/*
 *	InnerModel *inne;
	Qvec pose;
	getInit(Inermodel *innev_);
		inner=innev_;
	void copy(x,z,id)
	id = id_;
	psoe=inne->transtf("world",Qvec.vec3(x,0,z),"base"); 
 * 
 * 
 */

#endif

