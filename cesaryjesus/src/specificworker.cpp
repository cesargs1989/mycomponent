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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{

   this->inner= new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
  
  
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{

        timer.start ( Period );
        return true;
}


void SpecificWorker::compute()
{

    RoboCompDifferentialRobot :: TBaseState bState;

    differentialrobot_proxy->getBaseState(bState);
    inner->updateTransformValuesS("base", bState.x, 0, bState.z ,0, bState.alpha, 0);
          
    switch(state)
    {    
    case State::IDLE:
      laserRandon();
    if ( target.active )
      state = State::GIRO;
      if (lado == true)
	state = State::GOTO;
      break;
    
    case State::GIRO:
      
      directo();
      break;
    
    case State::GOTO:
      //qDebug()<<"entra2";
      gotoTarget();
      //if (obstacle()==false)
	
      break;
      
    case State::BUG:
      bug();
      break;

    default:
      break;
          }
  
  
}



void SpecificWorker::gotoTarget(){
   
    if( obstacle() == true)   // If ther is an obstacle ahead, then transit to BUG
   {
      state = State::BUG;
      return;
   }
     
    QVec rt = inner->transform("base", target.getPose(), "world");
    float dist = rt.norm2();
    float ang  = atan2(rt.x(), rt.z());
   if(dist < 100)          // If close to obstacle stop and transit to IDLE

  {
    state = State::IDLE;
    target.setActive(true);
   return;
  }

  float adv = dist;
  if ( fabs( rot) > 0.05 )
   adv = 0;

 }
void SpecificWorker::bug()

{

}

bool SpecificWorker::obstacle()
{

  	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        std::sort ( ldata.begin() +8, ldata.end()-8, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return     a.dist < b.dist;
        } ) ; //sort laser data from small t				
	if ( ldata[12].dist < threshold ) {
	  return true;
	}
	else 
	  return false;
}

bool SpecificWorker::targetAtSight()

{

}



void SpecificWorker::setPick ( const Pick &myPick )
{
        target.copy ( myPick.x, myPick.z );
        target.setActive ( true );
        girando = false;
	laserini = false;
}

void SpecificWorker::directo(){
  

        //QVec posdst;

        RoboCompDifferentialRobot :: TBaseState bState;
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        std::sort ( ldata.begin() +8, ldata.end()-8, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return     a.dist < b.dist;
        } ) ; //sort laser data from small to
        try {
                if ( target.active ) {
                        differentialrobot_proxy-> getBaseState ( bState );

                        posz=target.getPose().getItem ( 1 );
                        posx=target.getPose().getItem ( 0 );

                        if ( !girando ) {

                                X = cos ( bState.alpha ) * ( posx - bState.x ) + ( -sin ( bState.alpha ) * ( posz - bState.z ) );
                                Z = sin ( bState.alpha ) * ( posx - bState.x ) +  cos ( bState.alpha ) * ( posz - bState.x ) ;

                                angulo = atan2 ( X, Z );
				angulo=abs(angulo);

                                if ( angulo <= limRot ) {
                                        differentialrobot_proxy->stopBase();
                                        girando = true;
                                } else
                                        differentialrobot_proxy->setSpeedBase ( 0, angulo );
                        } else if ( girando ) {
                                distancia = sqrt ( ( ( posx - bState.x ) * ( posx - bState.x ) ) + ( ( posz - bState.z ) * ( posz - bState.z ) ) );
                                differentialrobot_proxy->setSpeedBase ( 500,0 );
                                if ( distancia <= limite ) {
                                        target.setActive ( false );
                                        girando=false;
					//state = State::IDLE;  
                                        differentialrobot_proxy->stopBase();
                                }
                        }
                        laserini=true;
			lado = true;
		  
		}
            			  
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
  
  
}

void SpecificWorker::laserRandon(){
          RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        std::sort ( ldata.begin() +8, ldata.end()-8, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return     a.dist < b.dist;
        } ) ; //sort laser data from small to
	if (laserini == true || !target.active){				

	    if ( ldata[12].dist < threshold ) {
		    //std::cout << ldata.front().dist << std::endl;
		    if ( ldata[12].angle > 0 ) {

			    differentialrobot_proxy->setSpeedBase ( 5, -rot );
			    usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
		    } else {
			    differentialrobot_proxy->setSpeedBase ( 5, rot );
			    usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
		    }
	    } else {
		    differentialrobot_proxy->setSpeedBase ( 500, 0 ); // velocidad robot
	    }
	}

}













