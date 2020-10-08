$ javac Vertex.java
$ javac Edge.java
$ javac Dijkstra.java
$ java Disjkstra

You should see this displayed in the terminal:
Shortest path between SanFrancisco and Boston: SanFrancisco -> SaltLakeCity -> Denver -> Omaha -> Chicago -> Pittsburgh -> NewYork -> Boston Distance: 756.0728003914514

$ javac Display.java
$ java Display

In the GUI:
Click on Compute All Euclidean Distances
Select SanFrancisco as the start city
Select Boston as the end city
Click on Draw Dijkstra's Path

***If you change the names of the start and end cities in the main method of Dijkstra.java and go through the same process above, you will get the correct path and distance between whichever cities you choose in the terminal. Or you can just Load/Reset in the GUI to see the paths between different cities if you do not care about seeing it displayed in the terminal.