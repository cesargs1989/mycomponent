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
        state=State::INIT;

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{


        innerModel=new InnerModel ( "/home/robocomp/robocomp/files/innermodel/simpleworld.xml" );
        timer.start ( Period );
        pick.setActive ( false );
        return true;
}


void SpecificWorker::compute()
{
//     const float threshold = 100; //millimeters
//     const float limiteRot = 0.005; //limite rotacion
//     float rot, xpick, zpick, xrobot, zrobot, rotTemp, valorAbsrot, puntoUno, puntoDos;
//     double x, z, dist;
//

        RoboCompDifferentialRobot::TBaseState bState;
        RoboCompLaser::TLaserData lData;

        try {
                differentialrobot_proxy->getBaseState ( bState );
                lData =laser_proxy->getLaserData();
                innerModel->updateTransformValues ( "base", bState.x, 0, bState.z, 0, bState.alpha, 0 );
                QVec ini;

                switch ( state ) {
                case State::INIT:
                        
                        if ( pick.active ) {
                                ini = QVec::vec3 ( bState.x, 0, bState.z );
                                linea = QLine2D ( ini, pick.getPose() );
                                state=State::GOTO;
                        }
                        break;

                case State::GOTO:
                        movement ( lData );
                        break;
                case State::BUG:
                        bug ( lData, bState );
                        break;
                case State::BUGINIT:
                        buginit ( lData, bState );
                        break;
                case State::END:
                        break;
                }
        } catch ( const Ice::Exception &e ) {
                std::cout << e << std::endl;
        };

}

void SpecificWorker::buginit ( const TLaserData& lData,const TBaseState& bState )
{
        QVec posicion = QVec::vec3 ( bState.x, 0., bState.z );
        distanciaAnterior = fabs ( linea.perpendicularDistanceToPoint ( posicion ) );
        if ( !obstacle ( lData ) ) {
                state=State::BUG;
                return;
        }

        try {
                differentialrobot_proxy->setSpeedBase ( 0, 0.5 );
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }
}

void SpecificWorker::bug ( const TLaserData &lData, const TBaseState& bState )
{
//   float vr;
//   float max_adv = 350.;
        float const alfa = log ( 0.1 ) / log ( 0.3 );
        float dist = obstaculoEnIzquierda ( lData );
        float diffToline = distanceToLine ( bState );

//   QVec tr = innerModel->transform("base", t.getPose(), "world");
//   float distancia = tr.norm2();
//
//   if(distancia < 280)
//   {
//     st= State::INIT;
//     t.setActive(false);
//     differentialrobot_proxy->stopBase();
//     return;
//   }

        if ( targetAtSight ( lData ) ) {
                state = State::GOTO;
                qDebug() << "Target visible: from BUG to GOTO";
                return;
        }
        if ( distanciaAnterior < 100 and diffToline < 0 ) {
                state = State::GOTO;
                qDebug() << "Crossing the line: from BUG to GOTO";
                return;
        }
        if ( obstacle ( lData ) ) {
                state = State::BUGINIT;
                qDebug() << "from BUG to BUGINIT";
                return;
        }
//   const float m = 1.f/1000.f;
//   const float n = -0.5;
//
//   float d = lData[10].dist;
//   if(d>160 || d<130)
//     vr = m * d + n;
//
//   float vadv = exp(-fabs(vr)*alfa) * max_adv;

        float k=0.1;  // pendiente de la sigmoide
        float vrot =  - ( ( 1./ ( 1. + exp ( -k* ( dist - 450. ) ) ) )-1./2. );		//sigmoide para meter vrot entre -0.5 y 0.5. La k ajusta la pendiente.
        float vadv = 350 * exp ( - ( fabs ( vrot ) * alfa ) ); 		//gaussiana para amortiguar la vel. de avance en funcion de vrot
        qDebug() << vrot << vadv;
        differentialrobot_proxy->setSpeedBase ( vadv ,vrot );


}


bool SpecificWorker::obstacle ( TLaserData lData )
{
        const int threshold = 280;
        std::sort ( lData.begin() + 35, lData.end()- 35, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
                return a.dist < b.dist;
        } );
        return ( lData[35].dist < threshold );

}

bool SpecificWorker::targetAtSight ( TLaserData lData )
{
  QPolygon poly;
	for ( auto l: lData )
	{
		QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		poly << p;
	}
	QVec targetInRobot = innerModel->transform("base", pick.getPose(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 150);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(pick.getPose().x(),pick.getPose().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = innerModel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point - QVec::vec3(220,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point + QVec::vec3(220,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poly.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
return true;

}

void SpecificWorker::movement ( const TLaserData& lData )
{
        QVec tr = innerModel->transform ( "base", pick.getPose(), "world" );

        float angle = atan2 ( tr.x(),tr.z() );
        float dist = tr.norm2();

        if ( dist < 280 ) {
                state= State::INIT;
                pick.setActive ( false );
                differentialrobot_proxy->stopBase();
                return;
        }

        if ( obstacle ( lData ) ==true ) {
//     differentialrobot_proxy->stopBase();
                state=State::BUGINIT;
                return;
        }

        // preguntar si obstaculo

        if ( fabs ( angle ) > 0.05 )
                dist = 0;
        if ( dist > 300 )
                dist = 300;

        try {
                differentialrobot_proxy->setSpeedBase ( dist, angle );
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;
        }

}

float SpecificWorker::distanceToLine ( const TBaseState& bState )
{
        QVec posi = QVec::vec3 ( bState.x, 0., bState.z );
        float distanciaEnPunto = fabs ( linea.perpendicularDistanceToPoint ( posi ) );
        float diff = distanciaEnPunto - distanciaAnterior;
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


void SpecificWorker::go ( const string& nodo, const float x, const float y, const float alpha )
{
        pick.copy ( x,y );
        pick.setActive ( true );
        state= State::INIT;

}

void SpecificWorker::turn ( const float speed )
{
        differentialrobot_proxy->setSpeedBase ( 0,speed );
}

bool SpecificWorker::atTarget()
{
        return !pick.isActive();
}

void SpecificWorker::stop()
{
        try {
                differentialrobot_proxy->stopBase();
        } catch ( const Ice::Exception &ex ) {
                std::cout << ex << std::endl;

        }

}

void SpecificWorker::setPick ( const Pick &myPick )
{
        qDebug() <<mypick.x<<mypick.z;
        pick.copy ( myPick.x, myPick.z );
        pick.setActive ( true );
        state = State::INIT;
//   girado=false;
}

