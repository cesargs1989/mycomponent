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
     const float threshold = 420; //millimeters
       float rot = 0.6;  //rads per second
        QVec posdst;
        float aux, aux2;
        float angulo;
        double distancia;
        float X, Z;
		int i;
        try {
                RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
                std::sort ( ldata.begin() +8, ldata.end()-8, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                        return     a.dist < b.dist;
                } ) ; //sort laser data from small to large distances using a lambda function.

                if ( target.active ) {
                        //RoboCompLaser::TData a;



                        RoboCompDifferentialRobot::TBaseState bState;
                        differentialrobot_proxy->getBaseState ( bState );

                        posdst = target.getPose(); //dir destino
                        angulo=0;
			distancia=0;
			qDebug()<< "PUNTOS: " <<  bState.x  << " - " << bState.z;
			
			
                        //R(x) +T(x,z);
                        Z = ( cos ( bState.alpha ) * bState.z ) + ( -sin ( bState.alpha )  * bState.x ) + (-posdst.y()) ;
                        X = ( sin ( bState.alpha ) * bState.z ) + ( cos ( bState.alpha )  * bState.x )+ posdst.x() ; 
                       

			qDebug()<< "raton: " << X << " - " << Z;
			
			
			angulo = atan2 ( X, Z);
			
			distancia=sqrt((X-bState.x)*(X-bState.x)+(Z-bState.z)*(Z-bState.z));
						
						
                        differentialrobot_proxy->setSpeedBase ( 0 ,  angulo-bState.alpha );
						
			qDebug()<< "distancia: " << distancia;


                        usleep ( 1000000 );
			i=0;

			differentialrobot_proxy->setSpeedBase ( distancia, 0);
			usleep ( 1000000 );

			target.setActive ( false );
			X = 0;
			Z = 0;

                } else {
                        differentialrobot_proxy->setSpeedBase ( 0 ,  0 );
                       /* if ( ldata[8].dist < threshold ) {
                                //std::cout << ldata.front().dist << std::endl;

                                if ( ldata[8].angle > 0 ) {

                                        differentialrobot_proxy->setSpeedBase ( 5, -rot );
                                        usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
                                } else {
                                        differentialrobot_proxy->setSpeedBase ( 5, rot );
                                        usleep ( rand() % ( 1500000-100000 + 1 ) + 100000 ); //random wait between 1.5s and 0.1sec
                                }
                        } else {
                                differentialrobot_proxy->setSpeedBase ( 500, 0 ); // velocidad robot
                        }

*/
                }



        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}

void SpecificWorker::setPick ( const Pick& myPick )
{
        target.copy ( myPick.x, myPick.z );
				qDebug()<< "raton: " << myPick.x << " - " << myPick.z;

        target.setActive ( true );
}

























