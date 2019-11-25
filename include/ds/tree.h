#ifndef TREE_H
#define TREE_H

#include <iostream>
#include <vector>

template <typename _Tp > class Tree;
template <typename _Tp >
class TreeNode
{
    TreeNode* parent = NULL;
    std::vector<TreeNode*> children;
   _Tp data;    
public:
   enum NodeType
   {
      ROOT = 0,
      CHILD = 1,
      ANSWER =2 //this means answer node, i.e. layer4 stored theta2
   };
private:
   NodeType type = CHILD;
   void SetNodeType(NodeType newtype) { type = newtype; }
public:
    TreeNode(TreeNode* nodeparent = NULL)
    {
       parent = nodeparent;
    }
    ~TreeNode()
    {
        for(int i = 0; i < children.size(); i++)
        {
            delete children[i];   
        }
        children.clear();
    }
    void SetParent(TreeNode* newparent);
    TreeNode* GetParent() ;
    const _Tp& GetData();
    void SetData(_Tp value);
    std::vector<TreeNode*>& GetChildren();
    NodeType GetNodeType(){ return type;}
    void SetType(NodeType newtype) { SetNodeType(newtype); }

template <typename _Ty >  friend class Tree;
};

template <typename _Tp >
class Tree
{
    // A root pointer of the whole tree
    TreeNode<_Tp>* root;
public:
    Tree(){ root = new  TreeNode<_Tp>; root->SetNodeType(TreeNode<_Tp>::ROOT); }
    ~Tree()
    {
        for(int i = 0; i < root->children.size(); i++)
        {
            delete root->children[i];
        }
        delete root;
    }
    //Get root is the first thing when use tree to get data
    TreeNode<_Tp>* GetRoot() ;
    //Add a node to the given parent
    TreeNode<_Tp>* AddNode(TreeNode<_Tp>* parent);
    //Delete all nodes in the tree.
    void ClearTree();

};

//TreeNode function definition
template <typename _Tp>
void TreeNode< _Tp>::SetParent( TreeNode< _Tp>* newparent)
{
    if(parent == NULL)
    {
        parent = newparent;
        return;
    }
    //Remove oneself from its parent
    std::vector<TreeNode< _Tp>*> copy;
    for(int i = 0; i < parent->GetChildren().size(); i++ )
    {
        if(parent->GetChildren()[i] == this)
        {}
        else
        {
            copy.push_back(parent->GetChildren()[i]);
        }
    }
    parent->GetChildren().clear();
    for (int i = 0; i < copy.size(); i++)
    {
        parent->GetChildren(copy[i]).push_back();
    }
    //Set a new parent
    parent = newparent;
}

template <typename _Tp>
TreeNode< _Tp>* TreeNode< _Tp>::GetParent()
{
    return parent;
}

template <typename _Tp>
const _Tp& TreeNode<_Tp>::GetData()
{
    return data;
}

template <typename _Tp>
void TreeNode< _Tp>::SetData(_Tp value)
{
    data = value;
}

template <typename _Tp>
std::vector<TreeNode< _Tp>*>& TreeNode< _Tp>::GetChildren()
{
    return children;
}

//Tree function definition
template <typename _Tp>
TreeNode<_Tp>* Tree< _Tp>::GetRoot()
{
    return root;
}
template <typename _Tp>
TreeNode<_Tp>* Tree< _Tp>::AddNode(TreeNode<_Tp>* parent)
{
    if(parent == NULL)
    {
        std::cout << "No parent of tree node is not allowed, addNode funtion failed !" << std::endl;
        return 0;
    }
   TreeNode<_Tp>* pNewNode= new TreeNode<_Tp>(parent);
   if(pNewNode == NULL)
   {
       std::cout << "Allocate space to  a new class TreeNode instance failed in function \"AddNode\" !" << std::endl;
       return 0;
   }
   parent->children.push_back(pNewNode);
   return pNewNode;
}

template <typename _Tp>
void Tree< _Tp>::ClearTree()
{
     for(int i = 0; i < root->children.size(); i++)
    {
        delete root->children[i];
    }
    root->children.clear();
}



#endif // TREE_H
