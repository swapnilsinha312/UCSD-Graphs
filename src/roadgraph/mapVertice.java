
package roadgraph;

import geography.GeographicPoint;

import java.util.HashSet;
import java.util.Set;

/**
* @author UCSD MOOC development team and YOU
* 
*         Class representing a vertex (or node) in our MapGraph
*
*/

class mapVertice implements Comparable 
{

	private HashSet<mapEdge> edges;
	private GeographicPoint currLoc;
	private double distance;
	private double aimDistance;


	mapVertice(GeographicPoint loc) 
	{
		currLoc = loc;
		edges = new HashSet<mapEdge>();
		distance = 0.0;
		aimDistance = 0.0;
	}

	void addEdge(mapEdge edge) 
	{
		edges.add(edge);
	}


	Set<mapVertice> getNeighbors() 
	{
		Set<mapVertice> neighbors = new HashSet<mapVertice>();
		for (mapEdge edge : edges) 
		{
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	GeographicPoint getLocation() 
	{
		return currLoc;
	}


	Set<mapEdge> getEdges() 
	{
		return edges;
	}


	public boolean equals(Object o) 
	{
		if (!(o instanceof mapVertice) || (o == null)) 
		{
			return false;
		}
		mapVertice node = (mapVertice) o;
		return node.currLoc.equals(this.currLoc);
	} 
	
	public String toString() {
		String toReturn = "[NODE at location (" + currLoc + ")";
		toReturn += " intersects streets: ";
		for (mapEdge e : edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	 
	public double getDistance() 
	{
		return this.distance;
	}

	public void setDistance(double distance) 
	{
		this.distance = distance;
	}

	public double getAimDistance() 
	{
		return this.aimDistance;
	}

	// set node distance (actual)
	public void setAimDistance(double aimDistance) 
	{
		this.aimDistance = aimDistance;
	}



	public int compareTo(Object o) 
	{
		mapVertice m = (mapVertice) o;
		return ((Double) this.getDistance()).compareTo((Double) m.getDistance());
	}

	
	 
	public mapVertice getNearestNeighbor() 
	{
		double minDist  = Double.POSITIVE_INFINITY;
		mapEdge shortestEdge = null;
		
		for (mapEdge edge : getEdges()) 
		{
			if (edge.getLength() < minDist ) 
				shortestEdge = edge;
			
		}
		return shortestEdge.getOtherNode(this);
	}
}