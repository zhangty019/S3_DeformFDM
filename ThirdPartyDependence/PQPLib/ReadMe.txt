//*******************************************************************************************
//*
//*	This folder contains the code that is extended from the basic PQP library 
//*	by adding the following functions:
//*
//*	1) find the closest point on a given mesh
//*	2) also find the triangle on the given mesh which contains the closest point
//*
//*******************************************************************************************
//	
// The following gives an example about how to use this extended PQP library
//	

#include "PQP/include/PQP.h"

float MinimalDistance(float x, float y, float z)
{
		//------------------------------------------------------------------
		//	build PQP model
		//------------------------------------------------------------------
		PQP_Model *m_pqp_model1 = new PQP_Model();
		m_pqp_model1->BeginModel();
		int i;
		PQP_REAL p1[3], p2[3], p3[3];
		for(i = 0; i< m_inMesh->face_N; i++)
		{
			int vid = m_inMesh->face[i][0];
			p1[0] = (PQP_REAL)(m_inMesh->vertex[vid][0]);
			p1[1] = (PQP_REAL)(m_inMesh->vertex[vid][1]);
			p1[2] = (PQP_REAL)(m_inMesh->vertex[vid][2]);
			vid = m_inMesh->face[i][1];
			p2[0] = (PQP_REAL)(m_inMesh->vertex[vid][0]);
			p2[1] = (PQP_REAL)(m_inMesh->vertex[vid][1]);
			p2[2] = (PQP_REAL)(m_inMesh->vertex[vid][2]);
			vid = m_inMesh->face[i][2];
			p3[0] = (PQP_REAL)(m_inMesh->vertex[vid][0]);
			p3[1] = (PQP_REAL)(m_inMesh->vertex[vid][1]);
			p3[2] = (PQP_REAL)(m_inMesh->vertex[vid][2]);
			m_pqp_model1->AddTri(p1,p2,p3,i);
		}
		m_pqp_model1->EndModel();


		//------------------------------------------------------------------
		//	compute minimal distance
		//------------------------------------------------------------------
		PQP_DistanceResult dres;	dres.last_tri = m_pqp_model1->last_tri;
		PQP_REAL p[3];
		p[0] = x;	p[1] = y;	p[2] = z;
		PQP_Distance(&dres,m_pqp_model1,p,0.0,0.0);

		float closestPt[3];	// closest point
		closestPt[0] = dres.p1[0];	closestPt[1] = dres.p1[1]; closestPt[2] = dres.p1[2];
		
		int minTriId = dres.last_tri->id;	//	closest triangle - note that ID index starts from zero

		float minDist = dres.Distance();	//	minimal distance



		//------------------------------------------------------------------
		//	free memory
		//------------------------------------------------------------------
		delete m_pqp_model1;

		return minDist;		
}		