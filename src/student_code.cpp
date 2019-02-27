#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
    Vector2D lerp(Vector2D x, Vector2D y, float t){
        return (1 - t) * x + t * y;
    }

    Vector3D lerp3D(Vector3D x, Vector3D y, double t){
        return (1 - t) * x + t * y;
    }
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
      std::vector<Vector2D> n;
      for(int i = 0; i < evaluatedLevels.back().size() - 1; i++){
          Vector2D lep = lerp(evaluatedLevels.back()[i], evaluatedLevels.back()[i+1], t);
          n.push_back(lep);
      }
      evaluatedLevels.push_back(n);
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
      std::vector<Vector3D> p;
      for(int i = 0; i < controlPoints.size(); ++i){
          p.push_back(evaluate1D(controlPoints[i], u));
      }
      return evaluate1D(p,v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
      std::vector<std::vector<Vector3D> > evaluatedLevels;
      evaluatedLevels.push_back(points);
      int c = 0;
      while(c != points.size() -1 ){
          std::vector<Vector3D> n;
          for(int i = 0; i < evaluatedLevels.back().size(); ++i){
              Vector3D lep = lerp3D(evaluatedLevels.back()[i], evaluatedLevels.back()[i+1], t);
              n.push_back(lep);
          }
          c++;
          evaluatedLevels.push_back(n);
      }
      return evaluatedLevels.back()[0];
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
      Vector3D n(0,0,0); // initialize a vector to store your normal sum
      HalfedgeCIter h = halfedge(); // Since we're in a Vertex, this returns a halfedge
      // pointing _away_ from that vertex
      h = h->twin(); // Bump over to the halfedge pointing _toward_ the vertex.
      // Now h->next() will be another edge on the same face,
      // sharing the central vertex.
      HalfedgeCIter h_orig = h; //save a copy of h
      do{
          Vector3D n1 = h->next()->vertex()->position - h->vertex()->position;
          Vector3D n2 = h->next() -> next() ->vertex()->position - h-> next() ->vertex()->position;
          n += cross(n1, n2);
          h = h->next()-> twin();
      }while(h != h_orig);
      return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
      if(!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()){
          HalfedgeIter h0 = e0->halfedge();
          HalfedgeIter h1 = h0->next();
          HalfedgeIter h2 = h1->next();
          HalfedgeIter h3 = h0->twin();
          HalfedgeIter h4 = h3->next();
          HalfedgeIter h5 = h4->next();
          HalfedgeIter h6 = h1->twin();
          HalfedgeIter h7 = h2->twin();
          HalfedgeIter h8 = h4->twin();
          HalfedgeIter h9 = h5->twin();

          VertexIter v0 = h0->vertex();
          VertexIter v1 = h3->vertex();
          VertexIter v2 = h2->vertex();
          VertexIter v3 = h5->vertex();

          EdgeIter e0 = h0->edge();
          EdgeIter e1 = h1->edge();
          EdgeIter e2 = h2->edge();
          EdgeIter e3 = h4->edge();
          EdgeIter e4 = h5->edge();


          FaceIter f0 = h0->face();
          FaceIter f1 = h3->face();

          h0->next() = h1;
          h0->twin() = h3;
          h0->vertex() = v3;
          h0->edge() = e0;
          h0->face() = f0;

          h1->next() = h2;
          h1->twin() = h7;
          h1->vertex() = v2;
          h1->edge() = e2;
          h1->face() = f0;

          h2->next() = h0;
          h2->twin() = h8;
          h2->vertex() = v0;
          h2->edge() = e3;
          h2->face() = f0;

          h3->next() = h4;
          h3->twin() = h0;
          h3->vertex() = v2;
          h3->edge() = e0;
          h3->face() = f1;

          h4->next() = h5;
          h4->twin() = h9;
          h4->vertex() = v3;
          h4->edge() = e4;
          h4->face() = f1;

          h5->next() = h3;
          h5->twin() = h6;
          h5->vertex() = v1;
          h5->edge() = e1;
          h5->face() = f1;
          
          h6->twin() = h5;
          h7->twin() = h1;
          h8->twin() = h2;
          h9->twin() = h4;

          h9->twin() = h4;
          h9->vertex() = v1;
          h9->edge() = e4;

          e0->halfedge() = h0;
          e1->halfedge() = h5;
          e2->halfedge() = h1;
          e3->halfedge() = h2;
          e4->halfedge() = h4;

          v0->halfedge() = h2;
          v1->halfedge() = h5;
          v2->halfedge() = h3;
          v3->halfedge() = h0;

          f0->halfedge() = h0;
          f1->halfedge() = h3;
      }
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if(!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {

          HalfedgeIter h0 = e0->halfedge();
          HalfedgeIter h1 = h0->next();
          HalfedgeIter h2 = h1->next();
          HalfedgeIter h3 = h0->twin();
          HalfedgeIter h4 = h3->next();
          HalfedgeIter h5 = h4->next();
          HalfedgeIter h6 = h1->twin();
          HalfedgeIter h7 = h2->twin();
          HalfedgeIter h8 = h4->twin();
          HalfedgeIter h9 = h5->twin();

          EdgeIter e0 = h0->edge();
          EdgeIter e1 = h1->edge();
          EdgeIter e2 = h2->edge();
          EdgeIter e3 = h4->edge();
          EdgeIter e4 = h5->edge();
          EdgeIter e5 = newEdge();
          EdgeIter e6 = newEdge();
          EdgeIter e7 = newEdge();
          
          e6->isNew = true;
          e7->isNew = true;
          
          VertexIter v0 = h0->vertex();
          VertexIter v1 = h3->vertex();
          VertexIter v2 = h2->vertex();
          VertexIter v3 = h5->vertex();

          FaceIter f0 = h0->face();
          FaceIter f1 = h3->face();
          FaceIter f2 = newFace();
          FaceIter f3 = newFace();
          
          HalfedgeIter temp = e0->halfedge();
          Vector3D x = temp->vertex()->position;
          Vector3D y = temp->twin()->vertex()->position;
          VertexIter mid = newVertex();
          mid->position = (x + y) / 2;
          mid->isNew = true;

          HalfedgeIter h10 = newHalfedge();
          HalfedgeIter h11 = newHalfedge();
          HalfedgeIter h12 = newHalfedge();
          HalfedgeIter h13 = newHalfedge();
          HalfedgeIter h14 = newHalfedge();
          HalfedgeIter h15 = newHalfedge();


          h0->next() = h13;
          h0->twin() = h10;
          h0->vertex() = v0;

          h1->next() = h12;
          h1->twin() = h6;
          h1->vertex() = v1;
          h1->face() = f2;

          h2->next() = h0;
          h2->twin() = h7;
          h2->vertex() = v2;
          h2->edge() = e2;
          h2->face() = f0;

          h3->next() = h15;
          h3->twin() = h11;
          h3->vertex() = v1;
          h3->edge() = e5;
          h3->face() = f1;

          h4->next() = h14;
          h4->twin() = h8;
          h4->vertex() = v0;
          h4->edge() = e3;
          h4->face() = f3;

          h5->next() = h3;
          h5->twin() = h9;
          h5->vertex() = v3;
          h5->edge() = e4;
          h5->face() = f1;

          h6->twin() = h1;
          h6->vertex() = v2;
          h6->edge() = e1;

          h7->twin() = h2;
          h7->vertex() = v0;
          h7->edge() = e2;

          h8->twin() = h4;
          h8->vertex() = v3;
          h8->edge() = e3;

          h9->twin() = h5;
          h9->vertex() = v1;
          h9->edge() = e4;

          h10->next() = h4;
          h10->twin() = h0;
          h10->vertex() = mid;
          h10->edge() = e0;
          h10->face() = f3;

          h11->next() = h1;
          h11->twin() = h3;
          h11->vertex() = mid;
          h11->edge() = e5;
          h11->face() = f2;

          h12->next() = h11;
          h12->twin() = h13;
          h12->vertex() = v2;
          h12->edge() = e7;
          h12->face() = f2;

          h13->next() = h2;
          h13->twin() = h12;
          h13->vertex() = mid;
          h13->edge() = e7;
          h13->face() = f0;

          h14->next() = h10;
          h14->twin() = h15;
          h14->vertex() = v3;
          h14->edge() = e6;
          h14->face() = f3;

          h15->next() = h5;
          h15->twin() = h14;
          h15->vertex() = mid;
          h15->edge() = e6;
          h15->face() = f1;

          mid->halfedge() = h10;
          v0->halfedge() = h0;
          v1->halfedge() = h3;
          v2->halfedge() = h12;
          v3->halfedge() = h14;


          e0->halfedge() = h0;
          e1->halfedge() = h1;
          e2->halfedge() = h2;
          e3->halfedge() = h4;
          e4->halfedge() = h5;
          e5->halfedge() = h3;
          e6->halfedge() = h14;
          e7->halfedge() = h12;

          f0->halfedge() = h0;
          f1->halfedge() = h3;
          f2->halfedge() = h11;
          f3->halfedge() = h10;

          return mid;
      }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {

      for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v){
          Vector3D original_position = v->position;
          Vector3D neighboring_sum(0,0,0);
          HalfedgeCIter h = v->halfedge()->twin();
          int c = 0;
          do{
              neighboring_sum += h->vertex()->position;
              h = h->next()-> twin();
              c++;
          }while(h != v->halfedge()->twin());
          double u = 0.0;
          if(c == 3) {
              u = 3.0 / 16.0;
          }
          else {
              u = 3.0 / (8.0 * c);
          }
          v->newPosition = (1-c*u) * original_position + u * neighboring_sum;
          v->isNew = false;
      }

      for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
          Vector3D edge_AB(0,0,0);
          Vector3D edge_CD(0,0,0);
          HalfedgeCIter h = e ->halfedge();
          HalfedgeCIter htw = h-> twin();
          edge_AB = h->vertex()->position + htw->vertex()->position;
          edge_CD = htw->next()->next()->vertex()->position + h->next()->next()->vertex()->position;
          e->newPosition = 3.0/8.0 * edge_AB + 1.0/8.0 * edge_CD;
          e->isNew = false;
      }

      for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd();){
          EdgeIter next = e;
          HalfedgeCIter h = e ->halfedge();
          next++;
          if(!h->vertex()->isNew && !h->twin()->vertex()->isNew){
              VertexIter newVertex = mesh.splitEdge(e);
              newVertex->newPosition = e->newPosition;
          }
          e = next;
      }

      for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd();){
          EdgeIter next = e;
          HalfedgeCIter h = e ->halfedge();
          next++;
          if(e->isNew){
              if((h->vertex()->isNew && !h->twin()->vertex()->isNew) || (!h->vertex()->isNew && h->twin()->vertex()->isNew)){
                  mesh.flipEdge(e);
              }
          }
          e = next;
      }

      for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++){
          v->position = v->newPosition;
      }
  }
}
