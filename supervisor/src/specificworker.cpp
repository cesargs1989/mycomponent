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


        iner = new InnerModel ( "/home/jesusuiano/robocomp/files/innermodel/simpleworld.xml" );
        //timer.start ( Period );
        current = 0;

        return true;
}

void SpecificWorker::compute()
{

        try {
                RoboCompDifferentialRobot::TBaseState bState;
                differentialrobot_proxy->getBaseState ( bState );
                qDebug() << bState.x << bState.z;
                iner->updateTransformValues ( "base",bState.x,0,bState.z,0,bState.alpha,0 );
                qDebug() << current << "current state";
                switch ( estado ) {
                case Estado::BUSCAR:
                        qDebug() << "State SEARCH" ;
                        if ( tag.getID() == current ) {
                                gotopoint_proxy->stop();
                                gotopoint_proxy->go ( "base", tag.getPose().x(), tag.getPose().z(),0 );
                                estado = Estado::ESPERAR; 
                                qDebug() <<"State change to WAIT";
                        } else
                                gotopoint_proxy->turn ( 0.6 );
                        break;
                case Estado::ESPERAR:
                        qDebug() << "State WAIT" ;
                        if ( gotopoint_proxy->atTarget() == true ) {
                                gotopoint_proxy->stop();
                                estado = Estado::BUSCAR;
                                qDebug() <<"State change to SEARCH";
                                current = ( current+1 ) %4;
                        } else if ( tag.cambiado() ) {
                                gotopoint_proxy->go ( "", tag.getPose().x(), tag.getPose().z(), 0 );
                        }
                        break;
                }

        } catch ( const Ice::Exception &e ) {
                std::cout << "Error reading from Camera" << e << std::endl;
        }
}



void SpecificWorker::newAprilTag ( const tagsList &tags )
{
        tag.copy ( tags[0].tx,tags[0].tz,tags[0].id );
}






