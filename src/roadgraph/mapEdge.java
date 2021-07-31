/**
 * 
 */
package roadgraph; 
import geography.GeographicPoint;

/**
 * @author UCSD Intermediate Programming MOOC team and YOU
 *
 * A directed edge in a map graph from Node start to Node end
 */
public class mapEdge 
{
	 
	private String roadName;
	private String roadType;
	private mapVertice start;
	private mapVertice end;
	private double length;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	
	mapEdge(String roadName, mapVertice n1, mapVertice n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	mapEdge(String roadName, String roadType, mapVertice f, mapVertice t) 
	{
		this(roadName, roadType, f,t, DEFAULT_LENGTH);
	}
	
	mapEdge(String roadName, String roadType, mapVertice f, mapVertice t, double length) 
	{
		this.roadName = roadName;
		start = f;
		end = t;
		this.roadType = roadType;
		this.length = length;
	}
	
	 mapVertice getEndNode() 
	 {
	   return end;
	}
	
	 
	GeographicPoint getStartPoint()
	{
		return start.getLocation();
	}
	
	 
	GeographicPoint getEndPoint()
	{
		return end.getLocation();
	}
	
	 
	double getLength()
	{
		return length;
	}
	 
	public String getRoadName()
	{
		return roadName;
	}
	
	 
	mapVertice getOtherNode(mapVertice node)
	{
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	} 
	
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}

}
