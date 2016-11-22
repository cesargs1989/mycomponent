
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
        state = State::INIT;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
        innerModel= new InnerModel ( "/home/salabeta/robocomp/files/innermodel/simpleworld.xml" );
        timer.start ( Period );

        return true;
}

void SpecificWorker::laserRandom ( int threshold, TLaserData ldata )
{
        float rot = 0.7;

        if ( ldata[12].dist < threshold ) {
                std::cout << ldata[12].dist << std::endl;
                if ( ldata[12].angle > 0 ) {
                        differentialrobot_proxy->setSpeedBase ( 5, -rot );
                        usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
                } else {
                        differentialrobot_proxy->setSpeedBase ( 5, rot );
                        usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
                }

        }
}

void SpecificWorker::compute()
{

        try {

                RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
                RoboCompDifferentialRobot::TBaseState bState;
                differentialrobot_proxy->getBaseState ( bState );
                innerModel->updateTransformValues ( "base", bState.x, 0, bState.z, 0, bState.alpha, 0 );
QVec ini(3);
                switch ( state ) {
                case State::INIT:
                        if ( pick.active ){
                                state=State::GOTO;
				linea = QLine2D(ini,pick.getPose());
			}
                        break;
                case State::GOTO:
                        movement ( ldata );
                        break;
                case State::BUGINIT:
                        buginit ( ldata , bState);
                        break;
                case State::BUG:
                        bug ( ldata, bState );
                        break;
                case State::END:
                        break;
                }

        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}

void SpecificWorker::movement ( const TLaserData &tLaser )
{
        QVec tr = innerModel->transform ( "base",pick.getPose(),"world" );

        float angulo = atan2 ( tr.x(),tr.z() );
        float distance = tr.norm2();


        if ( distance <= 100 ) {
                pick.setActive ( false );
                state= State::INIT;
                differentialrobot_proxy->stopBase();
                return;
        }

        if ( obstacle ( tLaser ) ) {
                state=State::BUGINIT;
                return;
        }


        if ( abs ( angulo ) > 0.05 ) distance = 0;

        if ( distance > 300 ) distance = 300;

        try {
                differentialrobot_proxy->setSpeedBase ( distance, angulo );
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }

}


void SpecificWorker::buginit ( const TLaserData& ldata, const TBaseState& bState)
{
   QVec posi = QVec::vec3(bState.x, 0., bState.z);
    distanciaAnterior = fabs(linea.perpendicularDistanceToPoint(posi));
        if ( !obstacle ( ldata ) ) {
                state = State::BUG;
                return;
        }
          try
  {
      differentialrobot_proxy->setSpeedBase(0, 0.3);
  }
  catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }

}


void SpecificWorker::bug ( const TLaserData &ldata,const TBaseState &bState)
{
        const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
	float diffToline = distanceToLine(bState);
        float distI = obstaculoEnIzquierda ( ldata );

    QVec tr = innerModel->transform ( "base",pick.getPose(),"world" );
    float distance = tr.norm2();
    if ( distance <= 200 )
    {
        pick.setActive ( false );
	qDebug() << "FINISH: BUG TO INIT";
        state= State::INIT;
        differentialrobot_proxy->stopBase();
	return;
    }
    
    //TODO girando hacia target (enfocando) si hay caja entra en bug
    
    if ( targetAtSight ( ldata ) && diffToline <= 0 )  // Target a la vista y terminando de bordear
     {
        state = State::GOTO;
 	qDebug() << "from BUG to GOTO";
        return;
     }
    
    if ( obstacle (ldata) ){ // Obstaculo
      state = State::BUGINIT;
      qDebug() << "from BUG to BUGINIT";
      return;
    }



        float vrot =  - ( ( 1./ ( 1. + exp ( - 0.1*( distI - 450. ) ) ) )-1./2. );

        float dist = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); // QLin2D

        differentialrobot_proxy->setSpeedBase ( dist ,vrot );

	        if ( targetAtSight ( ldata ) ) { //TODO CAMBIAR POR CROSSLINE
                state = State::GOTO;
                return;
        }

}




bool SpecificWorker::targetAtSight ( TLaserData ldata )
{
        QPolygon poly;
        for ( auto l: ldata ) {
                QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
                QPoint p ( r.x(),r.z() );
                poly << p;
        }
        QVec targetInRobot = innerModel->transform ( "base", pick.getPose(), "world" );
        float dist = targetInRobot.norm2();
        int veces = int ( dist / 150 ); //number of times the robot semilength fits in the robot-to-target distance
        float landa = 1./veces;

        QList<QPoint> points;
        points << QPoint ( pick.getPose().x(),pick.getPose().z() ); //Add target

        //Add points along lateral lines of robot
        for ( float i=landa; i<= 1.; i+=landa ) {
                QVec point = targetInRobot* ( T ) landa;
                QVec pointW = innerModel->transform ( "world", point ,"base" );
                points << QPoint ( pointW.x(), pointW.z() );

                pointW = innerModel->transform ( "world", point - QVec::vec3 ( 250,0,0 ), "base" );
                points << QPoint ( pointW.x(), pointW.z() );

                pointW = innerModel->transform ( "world", point + QVec::vec3 ( 250,0,0 ), "base" );
                points << QPoint ( pointW.x(), pointW.z() );

        }
        foreach ( QPoint p, points ) {
                if ( poly.containsPoint ( p , Qt::OddEvenFill ) == false )
                        return false;
        }
        return true;

}



float SpecificWorker::distanceToLine(const TBaseState& bState)
{
  QVec posi = QVec::zeros(3);
  posi.setItem(0,bState.x);
  posi.setItem(2,bState.z);
  float distanciaEnPunto = fabs(linea.perpendicularDistanceToPoint(posi));
  float diff = distanciaEnPunto- distanciaAnterior;
//  qDebug()<< diff<< "en crossedLine";
  
  distanciaAnterior = distanciaEnPunto;
  return diff;
}  


float SpecificWorker::obstaculoEnIzquierda ( const TLaserData& tlaser )
{
        const int laserpos = 85;
        float min = tlaser[laserpos].dist;
        for ( int i=laserpos-2; i<laserpos+2; i++ ) {
                if ( tlaser[i].dist < min )
                        min = tlaser[i].dist;
        }
        return min;
}

void SpecificWorker::stopRobot()
{

        try {
                differentialrobot_proxy->stopBase();
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}

bool SpecificWorker::obstacle ( TLaserData tLaser )
{


        std::sort ( tLaser.begin() +35, tLaser.end()-35, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return     a.dist < b.dist;
        } ) ; //sort laser data from small to large distances using a lambda function.

        return ( tLaser[35].dist < 320 );


}

///////////////////////////////////////////////777
//////////////////////////////////////////////////

void SpecificWorker::setPick ( const Pick &mypick )
{
        qDebug() <<mypick.x<<mypick.z;
        pick.copy ( mypick.x,mypick.z );
        pick.setActive ( true );
        state = State::INIT;
}


bool SpecificWorker::atTarget()
{
        return false;
}
void SpecificWorker::go ( const string& nodo, const float x, const float y, const float alpha )
{
    pick.copy(x,y);
    pick.setActive ( true );
    state = State::INIT;

}
void SpecificWorker::stop()
{
          try {
                differentialrobot_proxy->stopBase();
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}
void SpecificWorker::turn ( const float speed )
{

}