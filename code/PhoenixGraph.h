///////////////////////////////////////////////////////////////////////////////
// PhoenixGraph, Edge list based graph library.
// Copyright (C) 2006-2010  Anssi Gröhn / eNtity | entity at iki dot fi
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
// 
// 
////////////////////////////////////////////////////////////////////////////////
/// \mainpage
/// <H1>This is API documentation for PhoenixGraph Library.</H1>
///
/// \author Anssi Gr&ouml;hn / eNtity | entity at iki dot fi
/// \section General
///  There exists three different classes, namely 
///  <ul>
///    <li>CGraph
///    <li>CGraphNode
///    <li>CGraphEdge
///  </ul>
///   
///  CGraph consists of nodes (aka. vertices) and edges. Nodes are connected via 
///  edges. 
///
///  \section Using
///   All you need to do is copy header PhoenixGraph.h into your source directory and include it:
///  <pre>
///  \#include "PhoenixGraph.h"
///  </pre>
///
///  No additional compile flags are required.
//////////////////////////////////////////////////////////////////////////////// 
#ifndef __PhoenixGraph_h__
#define __PhoenixGraph_h__
////////////////////////////////////////////////////////////////////////////////
#include <stack>
#include <list>
#include <queue>
#include <algorithm>
#include <string.h>
#include <assert.h>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////
#define PHOENIX_API 
////////////////////////////////////////////////////////////////////////////////
namespace Phoenix
{
  namespace Core
  {
    
    /////////////////////////////////////////////////////////////////
    // Declaration so we can use graphnode in GraphEdge
    class CGraphNode;
    class CGraph;
    /////////////////////////////////////////////////////////////////
    /// \brief A class for an directed edge between two nodes in a graph.
    class PHOENIX_API CGraphEdge
    {
      //#define GRAPH_EDGE_IMPL(NAME)  NAME( CGraphNode *pFrom,  CGraphNode *pTo ) : CGraphEdge(pFrom, pTo)  {}
      friend class CGraphNode;
      friend class CGraph;
    protected:
      /// \brief A pointer to the node where this edge is leaving from.
      CGraphNode *m_pFrom;
      /// \brief A pointer to the node where this edge is arriving.
      CGraphNode *m_pTo;
      /// is this edge traversed
      bool	       m_bTraversed;
      /// cost/weight of this edge
      int		       m_iCost;
      ////////////////////
      /// Constructor, time complexity O(1).
      CGraphEdge( )
      {
	m_pFrom = NULL;
	m_pTo   = NULL;
	m_bTraversed = 0;
      }
      ////////////////////
      /// Parametrized constructor. Prohibits copying. Time complexity O(1)
      /// \param pFrom CGraphNode where edge starts
      /// \param pTo CGraphNode where edge leads to.
      CGraphEdge( CGraphNode *pFrom,  CGraphNode *pTo );

      ////////////////////
      /// \brief The destructor. Time complexity O(1)
      /// It is protected and should only be invoked by CGraph.
      virtual ~CGraphEdge()
      {
	m_pFrom = NULL;
	m_pTo = NULL;
	m_bTraversed = 0;
      }

    public:
      ////////////////////
      /// \brief Returns the from node. Time complexity O(1)
      /// \return A pointer to CGraphNode
      CGraphNode * GetFromNode()
      {
	return m_pFrom;
      }
      ////////////////////
      /// \brief Returns the to node. Time complexity O(1)
      /// \return A pointer to CGraphNode
      CGraphNode * GetToNode()
      {
	return m_pTo;
      }
      ////////////////////
      /// \brief Sets the node traveral state.Time complexity O(1)
      /// \param bFlag boolean 1 for traversed, 0 for not.
      inline void SetTraversed( bool bFlag )
      {
	m_bTraversed = bFlag;
      }
      ////////////////////
      /// \brief Returns the boolean for traversal state.Time complexity O(1)
      /// \return boolean 1 for traversed, 0 otherwise.
      inline bool IsTraversed()
      {
	return m_bTraversed;
      }
      ////////////////////
      /// \brief Returns the cost of this edge.Time complexity O(1)
      /// \return  the cost of this edge as int
      inline int GetCost()
      {
	return m_iCost;
      }
      ////////////////////
      /// \brief Assigns the cost for this edge.Time complexity O(1)
      /// \param iCost integer value for the cost.
      void SetCost( int iCost )
      {
	m_iCost = iCost;
      }

    };


    /*   Root Node */
    /*     /    |   |_  */
    /*    /     |     | */
    /*  Cam1   Cam2   Cam3 */
    /*   \_     |      | */
    /*     \_   |      | */
    /*       \  |     Group 2 */
    /*     Group 1     /  | */
    /*       / |      /   | */
    /*      /  |    Obj3  Obj4 */
    /*   Obj1 Obj2 */

    /// The Generic Graph Node  class. 
    /// 
    ///  In addition, the graph SHOULD resemble more of a tree than a fully-fledged
    ///  graph, but it is handy to have possibility for configurations like one
    ///  below: 
    ///  \dot
    ///  digraph RenderGraph {
    ///     root [ label="Root node" ];
    ///     cam1 [ label="Camera 1" ];
    ///     cam2 [ label="Camera 2" ];
    ///     cam3 [ label="Camera 3" ];
    ///     group1 [ label="Group 1" ];
    ///     group2 [ label="Group 2" ];
    ///     obj1 [label="Object 1" ];
    ///     obj2 [label="Object 2" ];
    ///     obj3 [label="Object 3" ];
    ///     obj4 [label="Object 4" ];
    ///
    ///     root -> cam1;
    ///     root -> cam2;
    ///     root -> cam3;
    ///     cam1 -> group1;
    ///     cam2 -> group1;
    ///     cam3 -> group2;
    ///     group1 -> obj1;
    ///     group1 -> obj2;
    ///     group2 -> obj3;
    ///     group2 -> obj4;
    ///  }
    ///
    ///  \enddot
    /// Where Cameras 1 and 2 have can have the same objects (Obj1 & Obj2) drawn, 
    /// whereas Objects Obj3 and Obj4 are visible only using Camera 3.

    typedef std::list< CGraphEdge * > EdgeListType;
    typedef std::list< CGraphNode * > NodeListType;
    ////////////////////////////////////////////////////////////////////////////////
    /// Graph node (vertex) class.
    class PHOENIX_API CGraphNode 
    {
      friend class CGraph;
      friend class CGraphEdge;
    protected:
      /// A list of pointers to CGraphEdge objects leaving from this node.
      EdgeListType m_lstLeaving;
      /// A list of pointers to CGraphEdge objects arriving to this node.
      EdgeListType m_lstArriving;
      /// A pointer to Cgraph where this node belongs to.

      /// Symbolic color value. 
      int			      m_iColor;
      /// Boolean flag indicating has this node been visited.
      int m_bVisited;
      /// Boolean flag indicating has this node been culled.
      int m_bCulled;
      /// Boolean flag indicating has this node been changed.
      int m_bChanged;

      /// A pointer to the graph object which contains this node.
      CGraph *      m_pGraph;  
      ////////////////////
      /// Default constructor. Time complexity O(1)
      CGraphNode() 
      {

	SetVisited(0);
	m_bCulled = 0;
	m_bChanged = 0;
	m_pGraph = NULL;
	m_iColor = 0;
    
      }
      ////////////////////
      /// Destructor. Should be invoked with extreme care since it only removes this node
      /// not the children nor does it fix the parent pointers of the children.
      /// Time complexity O(1)
      virtual ~CGraphNode() {}
    public:
      ////////////////////
      /// Copy constructor. Time complexity O(1)
      CGraphNode( const CGraphNode &Node );
      ////////////////////
      /// Returns the graph where node is in. Time complexity O(1)
      CGraph * GetGraph();
      ////////////////////
      /// Sets the status of visited flag.  Time complexity O(1)
      /// \param bFlag boolean value for visited (1 = visited, 0 = unvisited )
      void SetVisited(int bFlag );
      ////////////////////
      /// Returns boolean value indicating has this node been visited.  Time complexity O(1)
      /// \return boolean 1 if this node has been visited, 0 if not.
      int IsVisited();
      ////////////////////
      /// Sets this node culling status.  Time complexity O(1)
      /// \param bState boolean value 1 if culled, 0 if not culled.
      void SetCulled( int bState );
      ////////////////////
      /// Returns boolean value indicated is this node culled.  Time complexity O(1)
      /// \return boolean 1 if culled, 0 otherwise
      int IsCulled();
      ////////////////////
      /// Sets the changed flag value to bFlag.  Time complexity O(1)
      /// \param bFlag boolean value indicating the new state.
      void SetChanged( int bFlag );
      ////////////////////
      /// Returns value of the m_bChanged flag.  Time complexity O(1)
      /// \returns boolean value if state is 'changed'.
      int IsChanged();
      ////////////////////
      /// Returns the list of edges leaving from this node.  Time complexity O(1)
      EdgeListType &GetLeavingEdges();
      ////////////////////
      /// Returns the list of edges arriving to this node.  Time complexity O(1)
      EdgeListType &GetArrivingEdges();
      ////////////////////
      /// Returns true if this node has leaving edges.  Time complexity O(1)
      int HasLeavingEdges();
      ////////////////////
      /// returns true if this node has arriving edges.  Time complexity O(1)
      int HasArrivingEdges();
      ////////////////////
      /// Removes leaving edges from this node.  Time complexity O(|edges|)
      /// Edges are removed alse from nodes where they arrive.
      void RemoveLeavingEdges();
      ////////////////////
      /// Removes arriving edges from this node. Time complexity O(|edges|)
      /// Edges are removed alse from nodes where they lead to.
      void RemoveArrivingEdges();
      ////////////////////
      ///  Adds an edge from this to given node. Time complexity O(1)
      /// \param pTo A pointer to CGraphNode.
      template< class GRAPH_EDGE >
      GRAPH_EDGE * AddEdge( CGraphNode *pTo );
      ////////////////////
      /// Removes and edge leading from this node to given node.
      /// O(|edges|)
      /// \param pTo A pointer to CGraphNode.
      int DeleteEdgeTo( CGraphNode *pTo );
      ////////////////////
      ///  Sets the color of this node. Time complexity O(1)
      /// \param iColor integer color
      void SetColor( int iColor );
      ////////////////////
      ///  Returns the color of this node. Time complexity O(1)
      /// \returns int the color.
      int  GetColor();
      ////////////////////
      ///  Returns the In-degree of this node. Time complexity O(1)
      /// \returns the number of arriving edges.
      size_t GetInDegree();
      ////////////////////
      ///  Returns the Out-degree of this node. Time complexity O(1)
      /// \returns the number of leaving edges.
      size_t GetOutDegree();
    };
    /////////////////////////////////////////////////////////////////
    /// Graph class, consists of nodes (or vertices) and edges ( or arcs).
    class PHOENIX_API CGraph
    {
    public:  

      /// How graph is supposed to be traversed.
      typedef enum 
	{
	  BREADTH_FIRST      = 0,   // breadth-first traversal 
	  DEPTH_FIRST        = 1,   // depth-first traversal
	  DEPTH_FIRST_BY_EDGES     // depth-first, but with every edge is considered as a new branch of a tree.
	} GraphTraversalMode;

    protected:
      /// List of nodes.
      NodeListType m_lstNodes;
      /// List of edges
      EdgeListType m_lstEdges;

    public:  
      
      ////////////////////
      /// Destructor. Time complexity O( |nodes| + |edges| ).
      virtual ~CGraph()
      {    
	RemoveEdges();
	RemoveNodes();
      }
      ////////////////////
      /// Deletes a node.  Time complexity O( |nodes| + |edges| ).
      /// \param pNode Node to be deleted.
      void DeleteNode( CGraphNode *pNode );
      ////////////////////
      /// Adds an edge between pNodeFrom and pNodeTo. Time complexity O(1).
      /// Returns Pointer to edge if ok, NULL on error
      template< class GRAPH_EDGE>
      GRAPH_EDGE * AddEdge( CGraphNode *pNodeFrom, CGraphNode *pNodeTo);
      ////////////////////
      /// Removes an edge. Time complexity O(edges).
      void DeleteEdge( CGraphEdge *pEdge );
      ////////////////////
      /// Removes all leaving edges from given node. Time complexity O(edges).
      /// \param pNode Node, where leaving edges are removed.
      void RemoveLeavingEdgesFrom( CGraphNode *pNode );
      ////////////////////
      /// Removes all arriving edges from given node. Time complexity O(edges).
      /// \param pNode Node, where arriving edges are removed.
      void RemoveArrivingEdgesFrom( CGraphNode *pNode );
      ////////////////////
      //CGraphNode * SeekNodeByNameAndType( const char *szName, 
      //							   const R iType );

      ////////////////////
      /// Sets each node as unvisited. Time complexity O(nodes).
      void  SetNodesUnvisited();
      ////////////////////
      /// Sets each edge untraversed. Time complexity O(edges).
      void  SetEdgesUntraversed();
      ////////////////////
      /// Sets the color of all nodes. Time complexity O(nodes).
      /// \param nColor The color which all nodes will be colored.
      void  SetColor( unsigned int nColor );
      ////////////////////
      /// Creates a node of given type. Time complexity O(1).
      template <class Type>  inline Type * CreateNode()
      {
	Type *t = new Type();
	m_lstNodes.push_back(t);
	t->m_pGraph = this;
	return t;
      }
      ////////////////////
      /// Node count. Time complexity O(1).
      /// \returns number of nodes in graph.  
      inline const size_t GetNodeCount() const
      {
	return m_lstNodes.size();
      }
      ////////////////////
      /// Edge count. Time complexity O(1).
      /// \returns number of edges in graph.
      inline const size_t GetEdgeCount() const
      {
	return m_lstEdges.size();
      }
      ////////////////////
      /// Returns all nodes.  Time complexity O(1).
      /// \return list of nodes. 
      NodeListType & GetNodes() { return m_lstNodes;}
      ////////////////////      
      /// Returns all edges. Time complexity O(1).
      /// \return list of edges.
      EdgeListType & GetEdges () { return m_lstEdges; }

    protected:
      ////////////////////
      /// Removes all nodes from graph. Time complexity O(nodes)
      void RemoveNodes();
      ////////////////////
      /// Removes all edges from graph. Time complexity O(edges)
      void RemoveEdges();


    };
    
    template <class TRAVELLER_TYPE > 
    void TravelDF( CGraphNode *pStartNode, TRAVELLER_TYPE * pTraveller );
    
  } // namespace Core
} // namespace Phoenix
/////////////////////////////////////////////////////////////////
#define HAS_UNTRAVERSED_EDGES( N ) ( (unsigned int)(N->GetColor()) < N->GetOutDegree())
#define IS_VISITED( N ) ( N->GetColor() > 0 )
/////////////////////////////////////////////////////////////////
template< class GRAPH_EDGE>
inline GRAPH_EDGE *
Phoenix::Core::CGraph::AddEdge( CGraphNode *pNodeFrom, CGraphNode *pNodeTo )
{
  if ( pNodeFrom == NULL )
  {
    std::cerr << "FromNode is NULL" << std::endl;
    return NULL;
  }
  
  if ( pNodeTo == NULL )
  {
    std::cerr << "ToNode is NULL" << std::endl;
    return NULL;
  }
  
  assert ( (pNodeTo->m_pGraph == pNodeFrom->m_pGraph) && "Nodes belong to different graphs!");

  
  GRAPH_EDGE *pEdge = new GRAPH_EDGE( pNodeFrom, pNodeTo);
  pNodeFrom->GetLeavingEdges().push_back( pEdge );
  pNodeTo->GetArrivingEdges().push_back( pEdge );
  m_lstEdges.push_back(pEdge);
  return pEdge;
}
/////////////////////////////////////////////////////////////////
template< class GRAPH_EDGE >
inline GRAPH_EDGE *
Phoenix::Core::CGraphNode::AddEdge( CGraphNode *pTo )
{
  return m_pGraph->AddEdge<GRAPH_EDGE>( this, pTo );
}
/////////////////////////////////////////////////////////////////
using namespace Phoenix::Core;
/////////////////////////////////////////////////////////////////
inline 
Phoenix::Core::CGraphEdge::CGraphEdge( CGraphNode *pFrom,  CGraphNode *pTo )
{
  m_pFrom = pFrom;
  m_pTo   = pTo;

  // safety check, we do not allow links to be added to ourselves (why?)
  //assert( pFrom != pTo ); APPARENTLY useless.

  // Both nodes must be non-NULL
  assert( pFrom != NULL );
  assert( pTo != NULL );
  m_bTraversed = 0;
    
  // Both nodes must belong to same graph
  assert( pFrom->m_pGraph == pTo->m_pGraph );
    
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraph::DeleteNode( CGraphNode *pNode )
{
  NodeListType::iterator it;
  it = find(m_lstNodes.begin(), m_lstNodes.end(), pNode );
  
  if ( it != m_lstNodes.end())
  {

    RemoveLeavingEdgesFrom( pNode );
    RemoveArrivingEdgesFrom( pNode );
   
    m_lstNodes.erase(it);
    delete pNode;
  } 
  else 
  {
    std::cerr << "Node " << pNode 
	      << "is not part of this graph!" << std::endl;
  }
  
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraph::DeleteEdge( Phoenix::Core::CGraphEdge *pEdge)
{
  EdgeListType::iterator it;
  it = find(m_lstEdges.begin(), m_lstEdges.end(), pEdge );

  if ( it != m_lstEdges.end())
  {
    // Delete edge from Graph edge list 
    m_lstEdges.erase(it);
    
    EdgeListType &lstFromEdges = 
     pEdge->GetFromNode()->GetLeavingEdges();
    EdgeListType &lstToEdges = 
      pEdge->GetToNode()->GetArrivingEdges();

    // delete edge from the edge list of node where it is leaving
    it = find(lstFromEdges.begin(), lstFromEdges.end(), pEdge);
    if ( it!= lstFromEdges.end()) lstFromEdges.erase(it);

    // delete edge from the edge list of node it is pointing at
    it = find(lstToEdges.begin(), lstToEdges.end(), pEdge);
    if ( it!= lstToEdges.end()) lstToEdges.erase(it);

  }
  delete pEdge;
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraph::RemoveNodes()
{
  while( !m_lstNodes.empty())
  {
    CGraphNode *pTmpNode = m_lstNodes.front();
    m_lstNodes.pop_front();
    delete pTmpNode;
  }
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraph::RemoveEdges()
{
  while( !m_lstEdges.empty() )
  {
    CGraphEdge *pTmpEdge = m_lstEdges.front();
    m_lstEdges.pop_front();
    delete pTmpEdge;
  }
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraph::RemoveLeavingEdgesFrom( CGraphNode *pNode )
{
  EdgeListType::iterator it;
  while( !pNode->GetLeavingEdges().empty())
  {
    it = pNode->GetLeavingEdges().begin();
    DeleteEdge( *it );
  }
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraph::RemoveArrivingEdgesFrom( CGraphNode *pNode )
{
  EdgeListType::iterator it;
  while( !pNode->GetArrivingEdges().empty())
  {
    it = pNode->GetArrivingEdges().begin();
    DeleteEdge( *it );
  }
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraph::SetNodesUnvisited( )
{
  
  NodeListType::iterator it = m_lstNodes.begin();

  for( ; it != m_lstNodes.end(); it++)
  {
    (*it)->SetVisited(0);
  }
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraph::SetEdgesUntraversed( )
{
  EdgeListType::iterator it = m_lstEdges.begin();

  for( ; it != m_lstEdges.end(); it++)
  {
    (*it)->SetTraversed(0);
  }
}
/////////////////////////////////////////////////////////////////
inline void  
Phoenix::Core::CGraph::SetColor( unsigned int nColor )
{
  NodeListType::iterator it = m_lstNodes.begin();

  for( ; it != m_lstNodes.end(); it++) 
  {
    (*it)->SetColor(nColor);
  }
}
/////////////////////////////////////////////////////////////////
inline Phoenix::Core::CGraph *
Phoenix::Core::CGraphNode::GetGraph()
{
  return m_pGraph;
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraphNode::SetVisited(int bFlag )
{
  m_bVisited = bFlag;
}
/////////////////////////////////////////////////////////////////
inline int
Phoenix::Core::CGraphNode::IsVisited()
{
  return m_bVisited;
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraphNode::SetCulled( int bState )
{
  m_bCulled = bState;
}
/////////////////////////////////////////////////////////////////
inline int
Phoenix::Core::CGraphNode::IsCulled()
{
  return m_bCulled;
}
/////////////////////////////////////////////////////////////////
inline EdgeListType &
Phoenix::Core::CGraphNode::GetLeavingEdges()
{
  return m_lstLeaving;
}
/////////////////////////////////////////////////////////////////
inline EdgeListType &
Phoenix::Core::CGraphNode::GetArrivingEdges()
{
  return m_lstArriving;
}
/////////////////////////////////////////////////////////////////
inline 
int
Phoenix::Core::CGraphNode::HasLeavingEdges()
{
  return (m_lstLeaving.size() > 0 );
}
/////////////////////////////////////////////////////////////////
inline 
int
Phoenix::Core::CGraphNode::HasArrivingEdges()
{
  return (m_lstArriving.size() > 0 );
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraphNode::SetChanged( int bFlag )
{
  m_bChanged = bFlag;
}
/////////////////////////////////////////////////////////////////
inline int
Phoenix::Core::CGraphNode::IsChanged()
{
  return m_bChanged;
}
/////////////////////////////////////////////////////////////////
inline int 
Phoenix::Core::CGraphNode::DeleteEdgeTo( CGraphNode *pTo )
{
  EdgeListType::iterator it = this->GetLeavingEdges().begin();
  // For each leaving edge in this 
  for(;it!=this->GetLeavingEdges().end();it++)
  {
    // if edge points to pTo, we delete it.
    if ( (*it)->GetToNode() == pTo )
    {
      CGraphEdge *pEdge = *it;
      m_pGraph->DeleteEdge( pEdge );
      break;
    }
  }
  return 0;
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraphNode::RemoveLeavingEdges()
{
  m_pGraph->RemoveLeavingEdgesFrom( this );
}
/////////////////////////////////////////////////////////////////
inline void 
Phoenix::Core::CGraphNode::RemoveArrivingEdges()
{
  m_pGraph->RemoveArrivingEdgesFrom( this );
}
/////////////////////////////////////////////////////////////////
inline void
Phoenix::Core::CGraphNode::SetColor( int iColor )
{
  m_iColor = iColor;
}
/////////////////////////////////////////////////////////////////
inline int 
Phoenix::Core::CGraphNode::GetColor()
{
  return m_iColor;
}
/////////////////////////////////////////////////////////////////
inline size_t
Phoenix::Core::CGraphNode::GetInDegree()
{
  return m_lstArriving.size();
}
/////////////////////////////////////////////////////////////////
inline size_t
Phoenix::Core::CGraphNode::GetOutDegree()
{
  return m_lstLeaving.size();
}
/////////////////////////////////////////////////////////////////
#endif
