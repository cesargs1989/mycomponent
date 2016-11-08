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
        innerModel= new InnerModel ( "/home/jesusuiano/robocomp/files/innermodel/simpleworld.xml" );
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


                switch ( state ) {
                case State::INIT:
                        if ( pick.active )
                                state=State::GOTO;
                        break;
                case State::GOTO:
                        movement ( ldata );
                        break;
                case State::BUGINIT:
                        buginit ( ldata );
                        break;
                case State::BUG:
                        bug ( ldata );
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
		//angulo = abs ( angulo );
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

        if ( distance > 300 ) distance = 300;

        differentialrobot_proxy->setSpeedBase ( distance, angulo );

}


void SpecificWorker::buginit ( const TLaserData& ldata )
{
        if ( obstacle ( ldata ) == false ) {

                state = State::BUG;
                return;
        } else {

                differentialrobot_proxy->setSpeedBase (50, 0.5 );
        	
		}
}


void SpecificWorker::bug ( const TLaserData &ldata )
{
        const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte

        if ( targetAtSight ( ldata ) ) { //TODO CAMBIAR POR CROSSLINE
                state = State::GOTO;
                return;
        }

        if ( obstacle ( ldata ) ) {
                state = State::BUGINIT;
                return;
        }

        float distD = obstacleDerecha ( ldata );
        float distI = obstaculoEnIzquierda ( ldata );

        qDebug() << " derecha: "<<distD;
        qDebug() << " izquierda: "<< distI;
        //vr = -( 1.0/800 ) * dist + 0.5;

        float vrot =  - ( ( 1./ ( 1. + exp ( - ( distI - 450. ) ) ) )-1./2. );




        float dist = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); // QLin2D
        
        differentialrobot_proxy->setSpeedBase ( dist ,vrot/2 );


}




bool SpecificWorker::targetAtSight ( TLaserData ldata )
{
        QPolygon poly;
        for ( auto l: ldata ) {
                QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
                QPoint p ( r.x(),r.z() );
                poly<<p;

        }
        return poly.containsPoint ( QPoint ( pick.getPose().x(),pick.getPose().z() ), Qt::OddEvenFill );

}






float SpecificWorker::obstaculoEnIzquierda ( const TLaserData& tlaser )
{
        float min = tlaser[90].dist;
        for ( int i= 1; i<5; i++ ) {
                if ( tlaser[90+i].dist < min ) {
                        min = tlaser[90+i].dist;
                }
        }
        return min;
}

float SpecificWorker::obstacleDerecha ( const TLaserData& tlaser )
{
        float max = tlaser[90].dist;
        for ( int i= 1; i<5; i++ ) {
                if ( tlaser[90-i].dist > max ) {
                        max = tlaser[90-i].dist;
                }
        }
        return max;
}

void SpecificWorker::stopRobot()
{

        differentialrobot_proxy->stopBase();

}

bool SpecificWorker::obstacle ( TLaserData tLaser )
{


        std::sort ( tLaser.begin() +27, tLaser.end()-27, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return     a.dist < b.dist;
        } ) ; //sort laser data from small to large distances using a lambda function.

        return ( tLaser[27].dist < 420 );


}

void SpecificWorker::setPick ( const Pick &mypick )
{
        qDebug() <<mypick.x<<mypick.z;
        pick.copy ( mypick.x,-mypick.z );
        pick.setActive ( true );
        state = State::INIT;
}

