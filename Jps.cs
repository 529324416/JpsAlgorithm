using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IGrid{
	bool isMovable(Vector2Int pos);				// 检查目标点位是否可移动
    bool isWall(Vector2Int pos);					// 检查目标点是否是墙体或者是否不可移动
}

public interface IPriorityQueue<T>{
    bool notEmpty{get;}
    T get();
    void set(T obj, int priority);
    void clear();
}


public static class JpsUtils{
    public static readonly Vector2Int left = Vector2Int.left;
    public static readonly Vector2Int right = Vector2Int.right;
    public static readonly Vector2Int up = Vector2Int.up;
    public static readonly Vector2Int down = Vector2Int.down;
    public static readonly Vector2Int upRight = Vector2Int.one;
    public static readonly Vector2Int upLeft = new Vector2Int(-1, 1);
    public static readonly Vector2Int downRight = new Vector2Int(1, -1);
    public static readonly Vector2Int downLeft = new Vector2Int(-1, -1);
    public static Dictionary<Vector2Int, Vector2Int[]> verticalDirLut;

    public static void init(){
        verticalDirLut = new Dictionary<Vector2Int, Vector2Int[]>();
        Vector2Int[] horizontalLines = new Vector2Int[]{left, right};
        Vector2Int[] verticalLines = new Vector2Int[]{up, down};
        verticalDirLut.Add(left, verticalLines);
        verticalDirLut.Add(right, verticalLines);
        verticalDirLut.Add(up, horizontalLines);
        verticalDirLut.Add(down, horizontalLines);
    }

    /// <summary> 判断当前方向是否为一个直线方向 </summary>
    public static bool isLineDireaction(Vector2Int direaction){
        return direaction.x * direaction.y == 0;
    }
    public static int manhattan(Vector2Int p1, Vector2Int p2){
        /* 曼哈顿距离 */

        return Mathf.Abs(p1.x - p2.x) + Mathf.Abs(p1.y - p2.y);
    }
    public static int euler(Vector2Int p1, Vector2Int p2){
        /* 欧拉距离 */

        int dx = p1.x - p2.x;
        int dy = p1.y - p2.y;
        return dx * dx + dy * dy;
    }
}

public class JPS{

    public Dictionary<Vector2Int,JpsNode> Lut => lut;

    private Dictionary<Vector2Int, JpsNode> lut;
    private IPriorityQueue<JpsNode> nodes;
    private Vector2Int start;
    private Vector2Int end;
    private IGrid env;
    public JPS(){
        JpsUtils.init();
        lut = new Dictionary<Vector2Int, JpsNode>();
        // nodes = new IPriorityQueue<JpsNode>();
    }
    public Vector2Int[] find(IGrid env, Vector2Int s, Vector2Int e){

        this.lut.Clear();
        this.nodes.clear();
        this.env = env;
        this.start = s;
        this.end = e;

        this.lut.Add(s, new JpsNode(s, s, new Vector2Int[0], 0));            // 直接将起点加入到查找表

        // 起点是一个特殊的跳点，也是唯一一个全方向检测的跳点，其他跳点最多拥有三个方向
        Vector2Int[] dirs = new Vector2Int[]{
            JpsUtils.up,
            JpsUtils.down,
            JpsUtils.left,
            JpsUtils.right,
            JpsUtils.upLeft,
            JpsUtils.upRight,
            JpsUtils.downLeft,
            JpsUtils.downRight,
        };
        JpsNode S = new JpsNode(s, s, dirs, 0);
        nodes.set(S, 0);

        while(nodes.notEmpty){
            JpsNode node = nodes.get();
            if(node.pos == end)return completePath();
            foreach(Vector2Int d in node.direactions){
                if(JpsUtils.isLineDireaction(d)){
                    testLine(node.pos, d, node.cost);
                }else{
                    testDiagonal(node.pos, node.pos, d, node.cost);
                }
            }
        }
        return null;
    }
    public Vector2Int[] completePath(){

        Dictionary<Vector2Int, Vector2Int> cameFrom = new Dictionary<Vector2Int, Vector2Int>();
        HashSet<Vector2Int> closedSet = new HashSet<Vector2Int>();
        Queue<JpsNode> openSet = new Queue<JpsNode>();
        openSet.Enqueue(lut[end]);
        while(openSet.Count > 0){
            JpsNode node = openSet.Dequeue();
            closedSet.Add(node.pos);
            foreach(Vector2Int pos in node.parents){
                if(closedSet.Contains(pos))continue;
                cameFrom.Add(node.pos, pos);
                if(pos == start)return _trace(cameFrom);
                openSet.Enqueue(lut[pos]);
            }
        }
        return null;
    }
    private Vector2Int[] _trace(Dictionary<Vector2Int, Vector2Int> cameFrom){
        List<Vector2Int> path = new List<Vector2Int>();
        Vector2Int current = end;
        while(current != start){
            path.Add(current);
            current = cameFrom[current];
        }
        path.Add(start);
        return path.ToArray();
    }
    private void addPoint(Vector2Int parent, Vector2Int p, Vector2Int[] dirs, int fcost){
        /* 追加一个新的跳点 */

        if(lut.ContainsKey(p)){
            lut[p].parents.Add(parent);
        }else{
            JpsNode node = new JpsNode(parent, p, dirs, fcost);
            lut.Add(p, node);
            nodes.set(node, fcost + JpsUtils.euler(p, end));
        }
    }
    private void testDiagonal(Vector2Int parent, Vector2Int p, Vector2Int d, int fcost){
        /* 斜向跳跃准备, 主要是检查是否可以跳跃以及当前是否应该搜索跳点 */
        /* 检查x分量和y分量上是否存在碰撞体, 如果都不存在则不寻找强制邻居 */

        // 计算障碍物1和障碍物2的位置
        Vector2Int b1 = new Vector2Int(p.x + d.x, p.y);
        Vector2Int b2 = new Vector2Int(p.x, p.y + d.y);
        if(env.isMovable(b1)){
            if(env.isMovable(b2)){
                /* 情况1，B1和B2均为空，可以移动且本次移动不需要检测斜向的跳点 */
                p += d;
                if(env.isMovable(p)){
                    //新的位置不是障碍物
                    fcost ++;
                    if(p == end){
                        addPoint(parent, p, null, fcost);
                        return;
                    }
                    if(diagonalExplore(p, d, fcost)){
                        addPoint(parent, p, new Vector2Int[]{d}, fcost);
                        return;
                    }
                    testDiagonal(parent, p, d, fcost);          // 递归该函数
                }
            }else{
                // 情况3，b1可以移动，而b2不可移动
                p += d;
                if(env.isMovable(p)){
                    fcost ++;
                    if(p == end){
                        addPoint(parent, p, null, fcost);
                        return;
                    }
                    List<Vector2Int> dirs = testForceNeighborsInDiagonal(p, b2, d, Vector2Int.up);
                    if(diagonalExplore(p, d, fcost) || dirs.Count > 0){
                        dirs.Add(d);
                        addPoint(parent, p, dirs.ToArray(), fcost);
                        return;
                    }
                    testDiagonal(parent, p, d, fcost);
                }
            }
        }else{
            if(env.isMovable(b2)){
                // 情况4，b2可以移动，而b1不可移动

                p += d;
                if(env.isMovable(p)){
                    fcost ++;
                    if(p == end){
                        addPoint(parent, p, null, fcost);
                        return;
                    }
                    List<Vector2Int> dirs = testForceNeighborsInDiagonal(p, b1, d, Vector2Int.right);
                    if(diagonalExplore(p, d, fcost) || dirs.Count > 0){
                        dirs.Add(d);
                        addPoint(parent, p, dirs.ToArray(), fcost);
                        return;
                    }
                    testDiagonal(parent, p, d, fcost);
                }
            }else{
                // 情况2，两者均不可移动，什么都不做
                // code..
            }
        }
    }
    private List<Vector2Int> testForceNeighborsInDiagonal(Vector2Int X, Vector2Int B, Vector2Int D, Vector2Int mask){
        /* 检查给定地目标点和方向是否存在强制邻居, 该函数只适用于斜向搜索
        只要检测到一边就可以退出函数了，因为只可能存在一边 
        @X: 移动到的点X，
        @B：X点侧边的障碍物
        @D: X - parent 
        @mask: 方向遮罩 */

        List<Vector2Int> directions = new List<Vector2Int>();
        B += D * mask;
        if(env.isMovable(B)){
            directions.Add(B - X);
        }
        return directions;
    }
    private bool diagonalExplore(Vector2Int p, Vector2Int d, int cost){
        /* 朝着角点的分量方向进行探索 */
        bool _1 = testLine(p, new Vector2Int(d.x, 0), cost);
        bool _2 = testLine(p, new Vector2Int(0, d.y), cost);
        return _1 || _2;
    }
    private bool testLine(Vector2Int parent, Vector2Int d, int fcost){
        /* 从当前点p开始沿着直线方向d进行跳跃, 如果遇到了跳点, 则返回真值, 否则返回假值 
        该函数认为节点parent已经被访问过了 */

        Vector2Int p = parent + d;
        while(env.isMovable(p)){
            if(p == end){
                /* 找到终点时将终点加入openset */
                addPoint(parent, p, new Vector2Int[0], 0);
                return true;
            }
            fcost ++;
            List<Vector2Int> directions = testForceNeighborsInLine(p, d);
            if(directions.Count > 0){
                directions.Add(d);
                addPoint(parent, p, directions.ToArray(), fcost);
                return true;
            }
            p += d;
        }
        return false;
    }
    private List<Vector2Int> testForceNeighborsInLine(Vector2Int p, Vector2Int d){
        /* 检查给定的目标点和方向是否存在强制邻居, 该函数只适用于横纵搜索 
        @p: 点X
        @d: 方向PX，P为X的父节点*/

        List<Vector2Int> directions = new List<Vector2Int>();
        foreach(Vector2Int vd in JpsUtils.verticalDirLut[d]){
            Vector2Int blockPt = vd + p;
            if(env.isWall(blockPt) && env.isMovable(blockPt + d))directions.Add(vd + d);
        }
        return directions;
    }
}