# Advanced Graph Algorithms in C++

This project showcases an advanced and fully generic **Graph Module implemented in modern C++**, designed with Unreal Engine integration in mind.  
It demonstrates strong command of templates, STL containers, graph theory, and algorithmic problem-solving applied to game development.

---

## 🚀 Features

### ✅ **Generic Graph Template (`Grafo<T>`)**
The graph supports any vertex type using C++ templates and efficient adjacency lists.

### 🔍 **Graph Traversal Algorithms**
- **Depth-First Search (DFS)** – procedural exploration, AI scanning, zone detection  
- **Breadth-First Search (BFS)** – area expansion, influence maps, distance layers  

### 🕸️ **Connected Components**
- Identify all connected subgraphs  
- Count components  
- Useful for analyzing map topology or isolated gameplay areas  

### 🌉 **Bridge Edge Detection**
Implements logic similar to **Tarjan's algorithm** to find critical edges whose removal disconnects the graph.  
Useful for:
- Detecting chokepoints  
- Level design analysis  
- Vulnerable paths in strategy games  

### 🛣️ **Shortest Path: Dijkstra**
Standalone implementation of the classical shortest-path algorithm:
- NPC navigation  
- Point-to-point routing  
- Weighted movement systems  

### 🌲 **Minimum Spanning Tree: Prim**
Used for:
- Procedural dungeon generation  
- Optimal connection of gameplay rooms  
- Resource/building networks in strategy games  

### 🔄 **Route Optimization (TSP-style heuristic)**
Includes helper functions to compute efficient visit orders across multiple vertices:
- Patrol path generation  
- Collecting objectives  
- Optimized travel sequences  

---

## 🧠 Why This Project Matters

Modern game engines—including **Unreal Engine 5**—rely heavily on efficient data structures and algorithmic reasoning.  
This module demonstrates:

- Mastery of **modern C++** (templates, STL, iterators)
- Ability to implement **core graph algorithms manually**
- Skills relevant for **AI**, **pathfinding**, **level generation**, and **systems programming**
- A solid understanding of **performance-oriented** code suitable for real-time applications

---

