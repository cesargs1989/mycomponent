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
        const float threshold = 417; //millimeters
        float rot = 0.7;  //rads per second
        QVec posdst;
	float x=0, z=0, auxx=0, auxz=0;
	float catetoop=0, catetocont=0;
	float hipotenusa=0, radio;
	bool enc=false;
	

        try {
                RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
                std::sort ( ldata.begin() +8, ldata.end()-8, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                        return     a.dist < b.dist;
                } ) ; //sort laser data from small to large distances using a lambda function.

                if ( target.active ) {
						//RoboCompLaser::TData a;
                        differentialrobot_proxy->setSpeedBase ( 0, 0 );
			RoboCompDifferentialRobot::TBaseState bState;
			differentialrobot_proxy->getBaseState( bState);
			
			auxx = bState.x;
			auxz = bState.z;
                        qDebug() << bState.x << bState.z << bState.alpha;
			posdst = target.getPose(); //dir destino
			
			if (bState.x<0)
			  auxx=bState.x*-1;
			if (bState.z<0)
			  auxz=bState.z*-1;
			if (posdst.x()<0)
			  x=posdst.x()*-1;
			if (posdst.z()<0)
			  auxz=posdst.z()*-1;
			catetoop=auxz+z;
			catetocont=auxx+x;
			  
			qDebug() << "trololo" << catetoop << catetocont;
			
			
			
			hipotenusa = (catetocont * catetocont) + (catetoop * catetoop);
			hipotenusa = sqrt(hipotenusa);
			radio= catetoop / hipotenusa;
			
			qDebug() << "trololooooooooooooooooo radio" << radio;

			  
			differentialrobot_proxy->setSpeedBase ( 1, radio );
			
			if (radio>0.1){
			  qDebug() << "ya estoy enfocadoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo";	
			differentialrobot_proxy->setSpeedBase ( 0, 0 );
			enc=true;
			
		  }
			
						
                } else {

                        if ( ldata[8].dist < threshold ) {
                                std::cout << "laser" << ldata.front().dist << std::endl;

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

                }


        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}

void SpecificWorker::setPick ( const Pick& myPick )
{
        target.copy ( myPick.x, myPick.z );
        target.setActive ( true );
}

























