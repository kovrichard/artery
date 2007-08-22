#ifndef TESTCONNECTIONMANAGER_H_
#define TESTCONNECTIONMANAGER_H_

#include <vector>
#include <list>

#include "BaseConnectionManager.h"

/**
 * This ConnectionManager is fully 2D/3D-compatible and should 
 * replace the old ConnectionManager later.
 * But for now it stays a separate CM for performance- and 
 * bugtesting purposes.
 */
class TestConnectionManager : public BaseConnectionManager
{
protected:
	/**
	 * Represents a position inside a grid.
	 * This class provides some converting functions from a Coord
	 * to a GridCoord.
	 */
	class GridCoord
	{
	public:
		static const int UNDEFINED = 0;
		unsigned x;
		unsigned y;
		unsigned z;
		bool use2D;


		/**
		 * Initialize this GridCoord with the origin.
		 * Creates a 3-dimensional coord.
		 */
		GridCoord()
			:x(0), y(0), z(0), use2D(false) {};

		/**
		 * Initialize a 2-dimensional GridCoord with x and y.
		 */
		GridCoord(unsigned x, unsigned y)
			:x(x), y(y), z(UNDEFINED), use2D(true) {};

		/**
		 * Initialize a 3-dimensional GridCoord with x, y and z.
		 */
		GridCoord(unsigned x, unsigned y, unsigned z)
			:x(x), y(y), z(z), use2D(false) {};

		/**
		 * Simple copy-constructor.
		 */
		GridCoord(const GridCoord& o) {
			x = o.x;
			y = o.y;
			z = o.z;
			use2D = o.use2D;
        }

		/**
		 * Creates a GridCoord from a given Coord by dividing the
		 * x,y and z-values by "gridCellWidth".
		 * The dimension of the GridCoord depends on the Coord.
		 */
        GridCoord(const Coord& c, double gridCellWidth = 1.0) {
            x = static_cast<unsigned>(c.getX() / gridCellWidth);
            y = static_cast<unsigned>(c.getY() / gridCellWidth);
            z = static_cast<unsigned>(c.getZ() / gridCellWidth);
            use2D = c.is2D();
        }

        std::string info() const {
			std::stringstream os;
			if (use2D) {
				os << "(" << x << "," << y << ")";
			} else {
				os << "(" << x << "," << y << "," << z << ")";
			}
			return os.str();
		}

		friend bool operator==(const GridCoord& a, const GridCoord& b) {
			return a.x == b.x && a.y == b.y && a.z == b.z;
		}

		friend bool operator!=(const GridCoord& a, const GridCoord& b) {
			return !(a==b);
		}

		GridCoord operator=(const GridCoord& a) {
			x = a.x;
			y = a.y;
			z = a.z;
			use2D = a.use2D;
			return *this;
		}
	};

	/**
	 * Represents an unsorted set of GridCoords with minimalistic functions.
	 * It is a workaround because c++ doesn't come with an unsorted set.
	 */
	class CoordSet {
	protected:
		std::vector<GridCoord*> data;
		unsigned maxSize;
		unsigned size;
		unsigned current;

	public:
		/**
		 * This values count the primary and secondary Collisions of the
		 * hashtable.
		 * This values are for performance testing and can be deleted 
		 * at release versions.*/
		unsigned cols;
		unsigned sCols;

	protected:

		/**
		 * Tries to insert a GridCoord at the specified position.
		 * If the same Coord already exists there nothing happens.
		 * If the an other Coord already exists there calculate
		 * a new Position to isnert end recursively call this Method again.
		 * If the spot is empty the Coord is inserted.
		 */
		void insert(const GridCoord& c, unsigned pos, bool prim = false) {
			if(data[pos] == 0) {
				data[pos] = new GridCoord(c);
				size++;
			} else {
				if(*data[pos] != c) {
					if(prim)
						cols++;
					else
						sCols++;
					insert(c, (pos + 2) % maxSize);
				}
			}
		}

	public:
		/**
		 * Initializes the set (hastable) with the a specified size.
		 */
		CoordSet(unsigned size)
			:maxSize(size), current(0), cols(0), size(0), sCols(0)
		{
			data.resize(maxSize);
		}

		/**
		 * Delete every created GridCoord
		 */
		~CoordSet() {
			for(unsigned i = 0; i < maxSize; i++) {
				if(data[i] != 0) {
					delete data[i];
				}
			}
		}

		/**
		 * Adds a GridCoord to the set. If the a GridCoord with the same
		 * value already exists in the set nothing happens.
		 */
		void add(const GridCoord& c) {
			unsigned hash = (c.x * 10000 + c.y * 100 + c.z) % maxSize;
			insert(c, hash, true);
		}

		/**
		 * Returns the next GridCoord in the set.
		 * You can interate through the set only one time with this function!
		 */
		GridCoord* next() {
			for(;current < maxSize; current++) {
				if(data[current] != 0) {
					return data[current++];
				}
			}
			return 0;
		}

		/**
		 * Returns the number of GridCoords currently saved in this set.
		 */
		unsigned getSize() { return size; }

		/**
		 * Returns the maximum number of elements which can be stored inside
		 * this set. To prevent collisions the set should never be more
		 * than 75% filled.
		 */
		unsigned getmaxSize() { return maxSize; }
	};
	
	typedef std::vector<NicEntries> RowVector;
	typedef std::vector<RowVector> NicMatrix;
    typedef std::vector<NicMatrix> NicCube;

   /** @brief Registry of all Nics
    *
    * This matrix keeps all Nics according to their position.  It
    * allows to restrict the position update to a subset of all nics.
    */
    NicCube nicGrid;
    
    /**
     * Distance that helps to find a node under a certain
     * position. Can be larger then @see maxInterferenceDistance to
     * allow nodes to be placed into the same square if the playground
     * is too small for the grid speedup to work. */
    double findDistance;

    /**
     * further cached values
     */
    GridCoord gridDim;

protected:
	/**
	 * @brief This method is called by "registerNic()" after the nic has been
	 * registered. That means that the NicEntry for the nic has already been 
	 * created and added to nics map.
	 * 
	 * @param nicID - the id of the NicEntry
	 */
	virtual void registerNicExt(int nicID);
	
	
	/** 
	 * @brief Calculate interference distance
	 */
	virtual double calcInterfDist();
	
	/**
	 * @brief Updates the connections of the nic with "nicID".
	 * 
	 * This method is called by "updateNicPos()" after the 
	 * new Position is stored in the corresponding nic.
	 * 
	 * @param nicID the id of the NicEntry
	 * @param oldPos the old position of the nic
	 * @param newPos the new position of the nic
	 */
	virtual void updateConnections(int nicID, const Coord* oldPos, const Coord* newPos);
	
	/**
     * find the next larger coordinate in grid, return true if the
     * connections in this position should be updated.
     */
	bool increment(unsigned max, unsigned src, unsigned* target);
	
	/**
     * find the next smaller coordinate in grid, return true if the
     * connections in this position should be updated.
     */
    bool decrement(unsigned max, unsigned src, unsigned* target);
    
    /** @brief Manages the connections of a registered nic. */ 
    void updateNicConnections(NicEntries& nmap, NicEntry* nic);

    /**
     * check connections of a nic in the grid
     */
    void checkGrid(GridCoord& oldCell,
                   GridCoord& newCell,
                   int id);

    /**
     * Calculates the corresponding cell of a coordinate.
     */
    GridCoord getCellForCoordinate(const Coord& c);

    /**
     * Returns the NicEntries of the cell with specified
     * coordinate.
     */
    NicEntries& getCellEntries(GridCoord& cell);

	/**
	 * If the value is outside of its bounds (zero and max) this function
	 * returns -1 if useTorus is false and the wrapped value if useTorus is true.
	 * Otherwise its just returns the value unchanged.
	 */
    int wrapIfTorus(int value, unsigned max);

	/**
	 * Adds every direct Neighbor of a GridCoord to a union of coords.
	 */
    void fillUnionWithNeighbors(CoordSet& gridUnion, GridCoord cell);
public:
	/**
	 * @brief Constructor
	 **/
	Module_Class_Members(TestConnectionManager, BaseConnectionManager, 0);
	
	/**
	 * @brief Reads init parameters and calculates a maximal interfence
	 * distance
	 **/
	virtual void initialize(int stage);
};

#endif /*TESTCONNECTIONMANAGER_H_*/
