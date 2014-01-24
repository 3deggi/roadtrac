/**
 * Original Author:    Sandy Eggi
 * Created:   24.01.2014
 * 
 * Data structure for saving descriptors from contour
 **/

#ifndef HASH_HPP
#define HASH_HPP

#include <map>
#include <vector>
#include <set>
#include <fstream>
#include <list>
using namespace std;

#include "Blob.hpp"

typedef pair<unsigned, unsigned> item;
typedef list<item> litem;

class HashTable
{
public:
	typedef long desctype;

	list<blobseq*> *lpref;
	void Clear();
	void MatchID(blobseq* target);
	void Init(blobseq *target);
	void Update(blobseq *target);
	bool Set(const desctype index, const unsigned ptid, const unsigned relid);
	void Combine(int p1, int p2, int p3, int p4, int id, int ptid);
	void BuildDescriptor(desctype*key, unsigned p1, unsigned p2, unsigned p3, unsigned p4);

	typedef map<unsigned, list<unsigned> > elm;
	typedef map< unsigned, litem> crs;
	typedef pair<unsigned,unsigned> pairelm;
	typedef set<pairelm> pairset;
private:
	map<desctype, set<unsigned> > m_rel;
	map<desctype, pairset> m_pair;
};

void HashTable::BuildDescriptor(desctype* key, unsigned p1, unsigned p2, unsigned p3, unsigned p4)
{
  *key = (p1*729000)+(p2*8100)+(p3*90)+p4;
}

void HashTable::Clear()
{
	m_rel.clear();
}

bool HashTable::Set(const desctype index, const unsigned ptid, const unsigned relid)
{
	pair <unsigned, unsigned> test(ptid, relid);
	m_pair[index].insert(test);
	return true;
}

void HashTable::MatchID(blobseq* target) 
{
	if (!target)
	{
		return;
	}

	if (!target->contours||target->contours->total<MINPOINTS)
	{
		target->id=NOID;
		return;
	}
	
	unsigned idfound = NOID;

	map<unsigned, unsigned> result;
	unsigned maxweight =0;
	crs corres; //this is for setting the correspondence
	map<unsigned, map<unsigned, list<unsigned> > > delta;

	target->ComputeMassCenter();
	target->ComputeBoundingRect();
	target->ConstructRelevanceList();//registering the measurement list

	if (target->relevance.size()<4)
	{
		target->id=NOID;
		return;
	}

	int c=0;
	
	for (list<unsigned>::const_iterator it=target->relevance.begin();it!=target->relevance.end();c++)
	{
		unsigned p1 = (unsigned)*it;
		it++;
		if (it==target->relevance.end())
		{
			it = target->relevance.begin();
			unsigned p2 = (unsigned)*it;
			it++;
			unsigned p3 = (unsigned)*it;
			it++;
			unsigned p4 = (unsigned)*it;
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			map<desctype, set<pair<unsigned,unsigned> > >::const_iterator idx = m_pair.find(key);
			if (idx!=m_pair.end())
			{
				unsigned str1 = idx->first;
				for (set<pair<unsigned,unsigned> >::const_iterator itx = idx->second.begin();itx!=idx->second.end();++itx)
				{
					int id = (*itx).first;
					int ptid = (*itx).second;
					list<blobseq*>::iterator lt = lpref->begin();
					advance(lt, id);
					result[id]++;
					if (result[id]>maxweight)
					{
						maxweight = result[id];
						idfound=id;
						corres[id].push_back(pair<unsigned,unsigned>(ptid, 1));
						delta[id][abs(ptid-1)].push_back((unsigned)corres[id].size()-1);
					}
				}
			}
			break;
		}
		
		list<unsigned>::const_iterator pt = it;
		unsigned p2 = (unsigned)*it;
		it++;
		if (it==target->relevance.end())
		{
			it = target->relevance.begin();
			unsigned p3 = (unsigned)(*it);
			it++;
			unsigned p4 = (unsigned)*it;
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			map<desctype, set<pair<unsigned,unsigned> > >::const_iterator idx = m_pair.find(key);
			if (idx!=m_pair.end())
			{
				for (set<pair<unsigned,unsigned> >::const_iterator itx = idx->second.begin();itx!=idx->second.end();++itx)
				{	
					int id = (*itx).first;
					int ptid = (*itx).second;			
					list<blobseq*>::iterator lt = lpref->begin();
					advance(lt, id);
					result[id]++;
					if (result[id]>maxweight)
					{
						maxweight = result[id];
						idfound=id;
						corres[id].push_back(pair<unsigned,unsigned>(ptid, 0));
						delta[id][ptid].push_back((unsigned)corres[id].size()-1);
						
					}
				}
			}
			break;
		}
		
		unsigned p3 = (unsigned)*it;
		it++;
		if (it==target->relevance.end())
		{
			unsigned p4 = (unsigned)(*target->relevance.begin());
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			map<desctype, set<item> >::const_iterator idx = m_pair.find(key);
			if (idx!=m_pair.end())
			{				
				for (set<pair<unsigned,unsigned> >::const_iterator itx = idx->second.begin();itx!=idx->second.end();++itx)
				{
					int id = (*itx).first;
					int ptid = (*itx).second;
					list<blobseq*>::iterator lt = lpref->begin();
					advance(lt, id);
					result[id]++;
					if (result[id]>maxweight)
					{
						maxweight = result[id];
						idfound=id;
						corres[id].push_back(item(ptid, (unsigned)(target->relevance.size()-1)));
						delta[id][abs((int)(ptid-((unsigned)target->relevance.size()-1)))].push_back((unsigned)corres[id].size()-1);
					}
				}
			}
			break;
		}
		unsigned p4 = (unsigned)*it;
		desctype key;
		BuildDescriptor(&key,p1,p2,p3,p4);
		map<desctype, set<pair<unsigned,unsigned> > >::const_iterator idx = m_pair.find(key);
		if (idx!=m_pair.end())
		{
			for (set<item>::const_iterator itx = idx->second.begin();itx!=idx->second.end();++itx)
			{
				int id = (*itx).first;
				int ptid = (*itx).second;
				list<blobseq*>::iterator lt = lpref->begin();
				advance(lt, id);
				result[id]++;
				if (result[id]>maxweight)
				{
					maxweight = result[id];
					idfound=id;
					corres[id].push_back(pair<unsigned,unsigned>(ptid, c+2));
					delta[id][abs(ptid-(c+2))].push_back((unsigned)corres[id].size()-1);				
				}
			}
		}
		it=pt;
	}

	target->candidates = result;

	if (idfound>=lpref->size()||idfound<0)
		return;

	target->id = idfound;
	target->weight = maxweight;
	
	if (delta[idfound].size()>1)
	{
		map<unsigned, list<unsigned> >::iterator itdel = delta[idfound].begin();
		list<unsigned>::iterator delid = itdel->second.begin();
		for (;delid!=itdel->second.end();++delid)
		{
			litem::iterator p = corres[target->id].begin();
			advance(p, *delid);
			target->match.push_back(*p);
		}
	}

	delta.clear();
	result.clear();
	corres.clear();
}

void HashTable::Combine(int p1, int p2, int p3, int p4, int id, int ptid)
{
	for (int i=p1-STR_DIFF;i<=p1+STR_DIFF;i++)
	{
		for (int j=p2-STR_DIFF;j<=p2+STR_DIFF;j++)
		{
			for (int k=p3-STR_DIFF;k<=p3+STR_DIFF;k++)
			{
				for (int l=p4-STR_DIFF;l<=p4+STR_DIFF;l++)
				{
					desctype key;
					BuildDescriptor(&key,i,j,k,l);
					Set(key, id, ptid);
				}
			}
		}
	}
}

void HashTable::Init(blobseq *target)
{
	int c=0;
	for (list<unsigned>::const_iterator it=target->relevance.begin();it!=target->relevance.end();c++)
	{
		unsigned p1 = (unsigned)*it;
		it++;
		if (it==target->relevance.end())
		{
			it = target->relevance.begin();
			unsigned p2 = (unsigned)*it;
			it++;
			unsigned p3 = (unsigned)*it;
			it++;
			unsigned p4 = (unsigned)*it;
			Combine(p1, p2, p3, p4, target->id, 1);
			break;
		}
		unsigned p2 = (unsigned)*it;
		list<unsigned>::const_iterator pt = it;
		it++;
		if (it==target->relevance.end())
		{

			it = target->relevance.begin();
			unsigned p3 = (unsigned)*it;
			it++;
			unsigned p4 = (unsigned)*it;
			Combine(p1, p2, p3, p4, target->id, 0);
			break;
		}
		unsigned p3 = (unsigned)*it;
		it++;

		if (it==target->relevance.end())
		{
			unsigned p4 = (unsigned)(*target->relevance.begin());
			Combine(p1, p2, p3, p4, target->id, (int)(target->relevance.size()-1));
			break;
		}
		unsigned p4 = (unsigned)*it;
			
		Combine(p1, p2, p3, p4, target->id,  c+2);
		it = pt;
	}
}

void HashTable::Update(blobseq *target)
{
	int c=0;
	for (list<unsigned>::const_iterator it=target->relevance.begin();it!=target->relevance.end();c++)
	{
		unsigned p1 = (unsigned)*it;
		it++;
		if (it==target->relevance.end())
		{
			it = target->relevance.begin();
			unsigned p2 = (unsigned)*it;
			it++;
			unsigned p3 = (unsigned)*it;
			it++;
			unsigned p4 = (unsigned)*it;
				
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			Set(key, target->id,1);
			break;
		}
		unsigned p2 = (unsigned)*it;
		list<unsigned>::const_iterator pt = it;
		it++;
		if (it==target->relevance.end())
		{

			it = target->relevance.begin();
			unsigned p3 = (unsigned)*it;
			it++;
			unsigned p4 = (unsigned)*it;
				
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			Set(key, target->id, 0);
			break;
		}
		unsigned p3 = (unsigned)*it;
		it++;

		if (it==target->relevance.end())
		{
			unsigned p4 = (unsigned)(*target->relevance.begin());
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);
			Set(key, target->id,(unsigned)target->relevance.size()-1);
			break;
		}
			
			unsigned p4 = (unsigned)*it;
			desctype key;
			BuildDescriptor(&key,p1,p2,p3,p4);

			Set(key, target->id,c+2);
			
			it = pt;
	}
}

#endif
