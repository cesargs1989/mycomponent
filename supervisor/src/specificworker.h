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

	void newAprilTag(const tagsList &tags);

public slots:
	void compute(); 	

private:
    enum class Estado {BUSCAR,ESPERAR};
    InnerModel* iner;
    Estado estado = Estado::BUSCAR;
    int current = 0;
    struct tagR
    {
      mutable QMutex m;
      QVec pose;
      QVec poseAnt = QVec::zeros(3);
      int id;
      InnerModel* inner;
      bool cambiado()
      {
	if( (pose - poseAnt).norm2() > 100)
	{
	  poseAnt = pose;
	  return true;
	}
	return false;
      };
      
      void Init(InnerModel* inm){
		inner = inm;
      
      }
      void copy( float x, float z, int id_){
		QMutexLocker lm(&m);
		pose = inner->transform ("world",QVec::vec3(x,0,z),"rgbd");
		qDebug()<< x << z << "dir recibidos del april";
		qDebug()<< pose.x() << pose.z() << "dir recibidos del transform";
		id = id_;
      }
      
	  QVec getPose(){
		QMutexLocker lm(&m);
		return pose;
	  }
	  
      int getID(){
	  QMutexLocker lm(&m);
		return id;
      }
    }tag; 
};

#endif