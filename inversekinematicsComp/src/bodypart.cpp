/*
    Copyright (c) 2014 <copyright holder> <email>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "bodypart.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto
 */
BodyPart::BodyPart(InnerModel *in, Cinematica_Inversa *invkin, QString p, QString t, QStringList ml)
{ 
	inner 		= in; 	
	ik			= invkin;
	part		= p;
	motorList	= ml; 	
	tip			= t;
};

BodyPart::BodyPart(const BodyPart& bp) : part(bp.part), tip(bp.tip), motorList(bp.motorList), ik(bp.ik), inner(bp.inner), listaTargets(bp.listaTargets), jointStepList(bp.jointStepList)
{
}

BodyPart::BodyPart(BodyPart&& bp): BodyPart()
{
	swap(*this,bp);
}

BodyPart& BodyPart::operator=(BodyPart bp)
{
	swap(*this, bp);
    return *this;
}

void swap(BodyPart& first, BodyPart& second)
{
	qSwap(first.part,second.part);
	qSwap(first.tip,second.tip);
	first.motorList.swap(second.motorList);
	first.ik = second.ik;
	first.inner = second.inner;
	first.listaTargets.swap(second.listaTargets);
	first.jointStepList.swap(second.jointStepList);
	
}

