/*
 *     Copyright 2010, 2015, 2017 Julian de Hoog (julian@dehoog.ca),
 *     Victor Spirin (victor.spirin@cs.ox.ac.uk),
 *     Christian Clausen (christian.clausen@uni-bremen.de
 *
 *     This file is part of MRESim 2.3, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle =
 *     "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
 *     }
 *
 *     @incollection{spirin2015mresim,
 *       title={MRESim, a Multi-robot Exploration Simulator for the Rescue Simulation League},
 *       author={Spirin, Victor and de Hoog, Julian and Visser, Arnoud and Cameron, Stephen},
 *       booktitle={RoboCup 2014: Robot World Cup XVIII},
 *       pages={106--117},
 *       year={2015},
 *       publisher={Springer}
 *     }
 *
 *     MRESim is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the FreeSpace Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     MRESim is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License along with MRESim.
 *     If not, see <http://www.gnu.org/licenses/>.
 */
package environment;

import config.SimConstants;
import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import javax.imageio.ImageIO;

/**
 *
 * @author julh
 */
public class OccupancyGrid implements IntGrid {

    /**
     * FreeSpace means something traversable where communication would work and agents might be
     * Plane means no slope or hill or obstacle Slope means not traversable for every agent Hill
     * means not traversable for most of the agents Obstacle means not traversable (here this is
     * obstacle and barrier as the agent does not differenciate between that) SafeSpace ??
     * KnownAtBAse is set if the information about the cell if known at basestation GotRelayed ??
     * FinalTopologicalMap flag is set, that means that we will no longer consider this cell when
     * rebuilding a topological map - we will just reuse the partial topological map we already
     * have.
     */
    public enum OccGridBit {
        FreeSpace, Plane, Slope, Hill, Obstacle, SafeSpace, KnownAtBase, GotRelayed, FinalTopologicalMap
    }
    private byte[][] grid;
    public int height;
    public int width;

    // occupancy grid information
    private int cellsMarkedAsFreeAndRelayedAndNotKnownAtBase;
    private int cellsMarkedAsFreeAndKnownAtBase;
    private int cellsMarkedAsFree;
    /**
     * List of cells that are free, not known at base and are not being relayed these are the cells
     * that we are currently "responsible" for delivering to base
     */
    private HashMap<Point, Integer> cellsFreeNotKnownAtBaseNotRelayed;
    // log the changes, so we don't have to process every cell each time we merge grids
    private HashMap<Point, Integer> cellsChanged;

    //this is a flag that can be used to check if occupancy grid has changed since it was last set to 'false'
    //used primarily to decide if we need to rebuild topological map
    private int mapCellsChanged;
    private int hashCode = 0;
    private int[][] skeleton;
    private LinkedList<Point> skeletonList;

    public OccupancyGrid(int newWidth, int newHeight) {
        width = newWidth;
        height = newHeight;
        grid = new byte[width][height];
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                grid[i][j] = 0;
            }
        }

        cellsMarkedAsFreeAndRelayedAndNotKnownAtBase = 0;
        cellsMarkedAsFreeAndKnownAtBase = 0;
        cellsMarkedAsFree = 0;

        cellsFreeNotKnownAtBaseNotRelayed = new HashMap<Point, Integer>();
        cellsChanged = new HashMap<Point, Integer>();

        mapCellsChanged = SimConstants.MAP_CHANGED_THRESHOLD + 1;
        skeletonList = new LinkedList<>();
    }

    public OccupancyGrid copy() {
        OccupancyGrid copyGrid = new OccupancyGrid(width, height);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                copyGrid.setByte(i, j, getByte(i, j));
            }
        }
        copyGrid.cellsMarkedAsFreeAndKnownAtBase = cellsMarkedAsFreeAndKnownAtBase;
        copyGrid.cellsMarkedAsFreeAndRelayedAndNotKnownAtBase = cellsMarkedAsFreeAndRelayedAndNotKnownAtBase;
        copyGrid.cellsMarkedAsFree = cellsMarkedAsFree;

        copyGrid.cellsFreeNotKnownAtBaseNotRelayed = (HashMap<Point, Integer>) cellsFreeNotKnownAtBaseNotRelayed.clone();

        copyGrid.mapCellsChanged = mapCellsChanged;
        copyGrid.cellsChanged = (HashMap<Point, Integer>) cellsChanged.clone();
        copyGrid.hashCode = hashCode();
        return copyGrid;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (obj.getClass() != getClass()) {
            return false;
        }

        return Arrays.equals(grid, ((OccupancyGrid) obj).grid);
    }

    @Override
    public int hashCode() {
        if (this.hashCode != 0) {
            return this.hashCode;
        } else {
            int hash = 5;
            hash = 73 * hash + Arrays.deepHashCode(this.grid);
            return hash;
        }
    }

    public void saveToPNG(String filename) {
        try {
            // retrieve image
            BufferedImage bi = new BufferedImage(800, 600, BufferedImage.TYPE_INT_RGB);
            //Graphics g = bi.getGraphics();
            for (int i = 0; i < 800; i++) {
                for (int j = 0; j < 600; j++) {
                    if (freeSpaceAt(i, j)) {
                        bi.setRGB(i, j, Color.WHITE.getRGB());
                    } else if (obstacleAt(i, j)) {
                        bi.setRGB(i, j, Color.BLUE.getRGB());
                    } else {
                        bi.setRGB(i, j, Color.BLACK.getRGB());
                    }
                }
            }
            File outputfile = new File(filename);
            ImageIO.write(bi, "png", outputfile);
        } catch (IOException e) {
            System.err.println(this + ": " + e);
        }
    }

    //TODO: should be able to make this more efficient
    public LinkedList<Point> mergeGrid(OccupancyGrid partnerOccGrid, boolean withBaseStation) {
        LinkedList<Point> cellsUpdated = new LinkedList();
        if (this.hashCode() == partnerOccGrid.hashCode()) {
            return cellsUpdated;
        }
        this.skeleton = null;
        this.skeletonList.clear();
        this.hashCode = 0;
        int totalCellsTransferred = 0;
        int cellsSetKnownAtBase = 0;
        for (int i = 0; i < this.width; i++) {
            for (int j = 0; j < this.height; j++) {
                if (this.getByteNoRelay(i, j) != partnerOccGrid.getByteNoRelay(i, j)) {
                    totalCellsTransferred++;

                    // if the information is completely new, get relay status
                    // otherwise, we may be the relay!
                    if (this.getByte(i, j) == 0) {
                        if (partnerOccGrid.isGotRelayed(i, j)) {
                            this.setGotRelayed(i, j);
                        }
                        //this.setByte(i, j, (byte)(this.getByte(i,j) | partnerOccGrid.getByte(i,j)));
                    }


                    if (partnerOccGrid.safeSpaceAt(i, j)) {
                        if (partnerOccGrid.freeSpaceAt(i, j)) {
                            if (this.safeSpaceAt(i, j) && (this.obstacleAt(i, j))) {
                                //Both think it's safe space, partner thinks it's free, we think it's obstacle
                                this.setFreeSpaceAt(i, j);
                                this.setNoObstacleAt(i, j);
                            } else {
                                this.setFreeSpaceAt(i, j);
                                this.setNoObstacleAt(i, j);
                                this.setSafeSpaceAt(i, j);
                            }
                        }
                        if (partnerOccGrid.obstacleAt(i, j)) {
                            if (this.safeSpaceAt(i, j) && (!this.obstacleAt(i, j))) {
                                //Both think it's safe space, partner thinks it's obstacle, we think it's free
                            } else {
                                this.setNoFreeSpaceAt(i, j);
                                this.setObstacleAt(i, j);
                                this.setSafeSpaceAt(i, j);
                            }
                        }
                    } else {
                        if (partnerOccGrid.freeSpaceAt(i, j)) {
                            if (this.safeSpaceAt(i, j)) {
                                // Do nothing, safe space always overrides unsafe space
                            } else if (this.obstacleAt(i, j)) {
                                this.setFreeSpaceAt(i, j);
                            } else {
                                this.setFreeSpaceAt(i, j);
                            }
                        }
                        if (partnerOccGrid.obstacleAt(i, j)) {
                            if (this.safeSpaceAt(i, j)) {
                                // Do nothing, safe space always overrides unsafe space
                            } else if (this.freeSpaceAt(i, j)) {
                                //do nothing, free space has priority
                            } else {
                                this.setNoFreeSpaceAt(i, j);
                                this.setObstacleAt(i, j);
                            }
                        }
                    }
                    /*
                    if (partnerOccGrid.freeSpaceAt(i, j) && (!this.obstacleAt(i, j))) {
                        this.setFreeSpaceAt(i, j);
                    }
                    if (partnerOccGrid.obstacleAt(i, j) && !this.safeSpaceAt(i, j))
                        this.setObstacleAt(i, j);*/
                    if (partnerOccGrid.isKnownAtBase(i, j) && !this.isKnownAtBase(i, j)) {
                        cellsSetKnownAtBase++;
                        this.setKnownAtBase(i, j);
                    }

                    /*else {
                        this.setByte(i, j, (byte)(this.getByte(i,j) | partnerOccGrid.getByteNoRelay(i,j)));
                    }*/
                    if (withBaseStation) {
                        if (!this.isKnownAtBase(i, j)) {
                            cellsSetKnownAtBase++;
                            this.setKnownAtBase(i, j);
                        }
                    }
                    cellsUpdated.add(new Point(i, j));
                }
                assert (this.getByteNoRelay(i, j) == partnerOccGrid.getByteNoRelay(i, j));
            }
        }
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("Cells transerred: " + totalCellsTransferred + ", set known at base: " + cellsSetKnownAtBase);
        }
        return cellsUpdated;
    }


    public boolean legalMove(Point source, Point dest) {
        int dx = dest.x - source.x;
        int dy = dest.y - source.y;

        if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
            return directLinePossible(source, dest, true, false);
        }

        if (dx != 0 && dy != 0) {
            //diagonal move (one diagonal pixel must be free)
            return ((locationExists(source.x + dx, source.y) && !obstacleAt(new Point(source.x + dx, source.y)))
                    || (locationExists(source.x, source.y + dy) && !obstacleAt(new Point(source.x, source.y + dy))))
                    && locationExists(dest.x, dest.y) && !obstacleAt(new Point(dest.x, dest.y));
        } else {
            //horizontal or vertical move
            return locationExists(dest.x, dest.y) && !obstacleAt(new Point(dest.x, dest.y));
        }
    }

    /**
     * Is freeSpace/not obstacle and has unknown cell around (frontier-cell)
     *
     * @param xCoord
     * @param yCoord
     * @return
     */
    public boolean frontierCellAt(int xCoord, int yCoord) {
        return //(
                //freeSpaceAt(xCoord, yCoord) &&
                //!safeSpaceAt(xCoord, yCoord) &&
                //!obstacleAt(xCoord, yCoord)
                //) ||
                (freeSpaceAt(xCoord, yCoord)
                && !obstacleAt(xCoord, yCoord)
                && frontierBorderCellAt(xCoord, yCoord));
    }

    /**
     * is unknown at any surrounding cell
     *
     * @param xCoord
     * @param yCoord
     * @return
     */
    public boolean frontierBorderCellAt(int xCoord, int yCoord) {
        //if(!frontierCellAt(xCoord, yCoord))
        //    return false;

        for (int i = xCoord - 1; i <= xCoord + 1; i++) {
            for (int j = yCoord - 1; j <= yCoord + 1; j++) {
                if (emptyAt(i, j)) {
                    return true;
                }
            }
        }

        return false;
    }

    public boolean isInOpenSpace(int xCoord, int yCoord) {
        for (int i = xCoord - 1; i <= xCoord + 1; i++) {
            for (int j = yCoord - 1; j <= yCoord + 1; j++) {
                if (!freeSpaceAt(i, j)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * is unonown but in grid
     *
     * @param xCoord
     * @param yCoord
     * @return
     */
    public boolean emptyAt(int xCoord, int yCoord) {
        if (xCoord < 0) {
            return false;
        }
        if (yCoord < 0) {
            return false;
        }
        if (xCoord > width) {
            return false;
        }
        if (yCoord > height) {
            return false;
        }
        return (!freeSpaceAt(xCoord, yCoord)
                && //!safeSpaceAt(xCoord, yCoord) &&
                !obstacleAt(xCoord, yCoord));
    }

    public boolean freeSpaceAt(Point p) {
        return freeSpaceAt((int) p.getX(), (int) p.getY());
    }

    public boolean freeSpaceAt(int xCoord, int yCoord) {
        if (getBit(xCoord, yCoord, OccGridBit.FreeSpace.ordinal()) == 1) {
            assert (!obstacleAt(xCoord, yCoord));
            return true;
        } else {
            return false;
        }
    }

    //we must not be able to set it to True from outside
    public void setMapHasChangedToFalse() {
        mapCellsChanged = 0;
    }

    public boolean hasMapChanged() {
        return (mapCellsChanged > SimConstants.MAP_CHANGED_THRESHOLD);
    }

    public int getMapCellsChanged() {
        return mapCellsChanged;
    }

    public boolean isFinalTopologicalMapCell(int xCoord, int yCoord) {
        return (getBit(xCoord, yCoord, OccGridBit.FinalTopologicalMap.ordinal()) == 1);
    }

    public void setFinalTopologicalMapCell(int xCoord, int yCoord) {
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FinalTopologicalMap, 1);
    }

    public void unsetFinalTopologicalMapCell(int xCoord, int yCoord) {
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FinalTopologicalMap, 0);
    }

    public boolean isKnownAtBase(int xCoord, int yCoord) {
        return (getBit(xCoord, yCoord, OccGridBit.KnownAtBase.ordinal()) == 1);
    }

    public void setKnownAtBase(int xCoord, int yCoord) {
        if (!isKnownAtBase(xCoord, yCoord) && freeSpaceAt(xCoord, yCoord)) {
            cellsMarkedAsFreeAndKnownAtBase++;
            if (isGotRelayed(xCoord, yCoord)) {
                cellsMarkedAsFreeAndRelayedAndNotKnownAtBase--;
            } else if (freeSpaceAt(xCoord, yCoord)) {
                Integer success = cellsFreeNotKnownAtBaseNotRelayed.remove(new Point(xCoord, yCoord));
                if (success == null) {
                    if (SimConstants.DEBUG_OUTPUT) {
                        System.out.println("@@@@@@@@@@ Tried to remove cellsFreeNotKnownAtBaseNotRelayed element "
                                + "that is not in the list! xCoord = " + xCoord + ", yCoord = " + yCoord);
                    }
                }
            }
        }
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.KnownAtBase, 1);
    }

    public boolean isGotRelayed(int xCoord, int yCoord) {
        return getBit(xCoord, yCoord, OccGridBit.GotRelayed.ordinal()) == 1;
    }

    // Marks this cell as being relayed to base by another robot
    // Used in UtilExploration.
    public void setGotRelayed(int xCoord, int yCoord) {
        setGotRelayed(xCoord, yCoord, true);
    }

    public void setGotRelayed(int xCoord, int yCoord, boolean updateOwnedCellsList) {
        if (!isGotRelayed(xCoord, yCoord) && freeSpaceAt(xCoord, yCoord) && !isKnownAtBase(xCoord, yCoord)) {
            cellsMarkedAsFreeAndRelayedAndNotKnownAtBase++;
            if (updateOwnedCellsList && freeSpaceAt(xCoord, yCoord) && !isKnownAtBase(xCoord, yCoord)) {
                Integer success = cellsFreeNotKnownAtBaseNotRelayed.remove(new Point(xCoord, yCoord));
                if (success == null) {
                    if (SimConstants.DEBUG_OUTPUT) {
                        System.out.println("@@@@@@@@@@ Tried to remove cellsFreeNotKnownAtBaseNotRelayed element "
                                + "that is not in the list! xCoord = " + xCoord + ", yCoord = " + yCoord);
                    }
                }
            }
        }
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.GotRelayed, 1);
    }

    // Marks this cell as NOT being relayed to base by another robot
    // Used in UtilExploration.
    public void setGotUnrelayed(int xCoord, int yCoord) {
        if (isGotRelayed(xCoord, yCoord) && freeSpaceAt(xCoord, yCoord) && !isKnownAtBase(xCoord, yCoord)) {
            cellsMarkedAsFreeAndRelayedAndNotKnownAtBase--;
            cellsFreeNotKnownAtBaseNotRelayed.put(new Point(xCoord, yCoord), 1);
        }
        assert (cellsMarkedAsFreeAndRelayedAndNotKnownAtBase >= 0);
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.GotRelayed, 0);
    }

    public int getNumFreeRelayedCells() {
        return cellsMarkedAsFreeAndRelayedAndNotKnownAtBase;
    }

    public int getNumFreeCellsKnownAtBase() {
        return cellsMarkedAsFreeAndKnownAtBase;
    }

    public int getNumFreeCells() {
        return cellsMarkedAsFree;
    }

    public void setFreeSpaceAt(int xCoord, int yCoord) {
        boolean wasObstacle = obstacleAt(xCoord, yCoord);
        if (!wasObstacle) { //Why?
            if (!freeSpaceAt(xCoord, yCoord)) {

                if (!wasObstacle) {
                    mapCellsChanged++; //only count map changes if we learn previously unknown map cell
                }                // We only recalculate topological map if some new cells of the occupancy grid have been learned.
                cellsMarkedAsFree++;
                if (isKnownAtBase(xCoord, yCoord)) {
                    cellsMarkedAsFreeAndKnownAtBase++;
                } else if (isGotRelayed(xCoord, yCoord)) {
                    cellsMarkedAsFreeAndRelayedAndNotKnownAtBase++;
                } else {
                    cellsFreeNotKnownAtBaseNotRelayed.put(new Point(xCoord, yCoord), 1);
                }

            }
            setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FreeSpace, 1);
        }
    }

    public void setNoFreeSpaceAt(int xCoord, int yCoord) {
        if (freeSpaceAt(xCoord, yCoord)) {
            //mapCellsChanged++;
            cellsMarkedAsFree--;
            if (isKnownAtBase(xCoord, yCoord)) {
                cellsMarkedAsFreeAndKnownAtBase--;
            } else if (isGotRelayed(xCoord, yCoord)) {
                cellsMarkedAsFreeAndRelayedAndNotKnownAtBase--;
            } else {
                Integer success = cellsFreeNotKnownAtBaseNotRelayed.remove(new Point(xCoord, yCoord));
                if (success == null) {
                    if (SimConstants.DEBUG_OUTPUT) {
                        System.out.println("@@@@@@@@@@ Tried to remove cellsFreeNotKnownAtBaseNotRelayed element "
                                + "that is not in the list! xCoord = " + xCoord + ", yCoord = " + yCoord);
                    }
                }
            }
        }
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FreeSpace, 0);
    }

    public void setNoObstacleAt(Point coord) {
        setNoObstacleAt(coord.x, coord.y);
    }

    public void setNoObstacleAt(int xCoord, int yCoord) {
        try {
            //if (obstacleAt(xCoord, yCoord)) mapCellsChanged++;
            setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.Obstacle, 0);
            setFreeSpaceAt(xCoord, yCoord);
        } catch (ArrayIndexOutOfBoundsException e) {
            System.err.println(this.toString() + "Error: ArrayIndexOutOfBoundsException.  Did not set as no obstacle.");
        }
    }

    public boolean safeSpaceAt(int xCoord, int yCoord) {
        return getBit(xCoord, yCoord, OccGridBit.SafeSpace.ordinal()) == 1;
    }

    public void setSafeSpaceAt(int xCoord, int yCoord) {
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.SafeSpace, 1);
        // safe space has to also be free space
        setFreeSpaceAt(xCoord, yCoord);
    }

    public void setNoSafeSpaceAt(int xCoord, int yCoord) {
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.SafeSpace, 0);
        // safe space has to also be free space
        //setFreeSpaceAt(xCoord, yCoord);
    }

    public boolean obstacleAt(Point p) {
        return obstacleAt((int) p.getX(), (int) p.getY());
    }

    public boolean obstacleAt(int xCoord, int yCoord) {
        return getBit(xCoord, yCoord, OccGridBit.Obstacle.ordinal()) == 1;
    }

    public void setObstacleAt(int xCoord, int yCoord) {
        try {
            //if (!obstacleAt(xCoord, yCoord) && freeSpaceAt(xCoord, yCoord)) mapCellsChanged++;
            setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.Obstacle, 1);
            setNoFreeSpaceAt(xCoord, yCoord);
        } catch (ArrayIndexOutOfBoundsException e) {
            System.err.println(this.toString() + "Error: ArrayIndexOutOfBoundsException.  Did not set as obstacle.");
        }
    }

    // List of cells that are free, not known at base and are not being relayed
    // these are the cells that we are currently "responsible" for delivering to base
    // Used in UtilityExploration to decide who should be the new agent responsible for delivery of the map cells
    public Collection<Point> getOwnedCells() {
        return cellsFreeNotKnownAtBaseNotRelayed.keySet();
    }

    // Makes our robot not responsible for delivery of any cells to the base station
    // Returns number of cells affected
    // Used in UtilityExploration
    public int setOwnedCellsRelayed() {
        getOwnedCells().stream().forEach((point) -> {
            setGotRelayed(point.x, point.y, false);
        });
        int counter = getOwnedCells().size();
        resetOwnedCells();
        return counter;
    }

    public LinkedList<Point> pointsAlongSegment(Point a, Point b) {
        LinkedList<Point> pts = new LinkedList<Point>();

        for (int i = Math.min(a.x, b.x); i <= Math.max(a.x, b.x); i++) {
            for (int j = Math.min(a.y, b.y); j <= Math.max(a.y, b.y); j++) {
                if (distPointToLine(a, b, new Point(i, j)) < 0.5) {
                    pts.add(new Point(i, j));
                }
            }
        }

        return pts;
    }

    // Used for UtilityExploration
    private void resetOwnedCells() {
        cellsFreeNotKnownAtBaseNotRelayed.clear();
    }

    public byte getByte(int x, int y) {
        return grid[x][y];
    }

    public byte getByteNoRelay(int x, int y) {
        return (byte) (grid[x][y] & ~(1 << OccGridBit.GotRelayed.ordinal())
                & ~(1 << OccGridBit.FinalTopologicalMap.ordinal()));
    }

    public int getBit(int xCoord, int yCoord, int bit) {
        try {
            if ((grid[xCoord][yCoord] & (byte) (Math.pow(2, bit))) > 0) {
                return 1;
            } else {
                return 0;
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            throw new ArrayIndexOutOfBoundsException("ERROR: Array index out of bounds at x=" + xCoord + ", y=" + yCoord + ".");
        }
    }

    public String toString(int xCoord, int yCoord) {
        String bitString = new String();
        for (int i = 0; i < 8; i++) {
            bitString = bitString.concat(Integer.toString(getBit(xCoord, yCoord, i)));
        }
        return (this.toString() + "Byte at " + xCoord + ", " + yCoord + " has value " + bitString);
    }

    @Override
    public String toString() {
        return ("[OccupancyGrid] ");
    }

    public boolean locationExists(int x, int y) {
        return (x < width && x >= 0 && y < height && y >= 0);
    }

    /**
     * Checks if there is a line from source to dest that doesn't go through obstacles. Note: by
     * current implementation lines are possible through unknown space. To change this only a slight
     * tweak in second part of if statement needed, i.e. change to: && !freeSpaceAt(i,j)
     *
     * @param source Start
     * @param dest Goal
     * @param allowUnknown
     * @param showErrors display Errors as X on the ExplorationImage
     * @return true if direct line possible considering the knowledge of the agent, false if not
     */
    public boolean directLinePossible(Point source, Point dest, boolean allowUnknown, boolean showErrors) {
        for (int i = Math.min(source.x, dest.x) + 1; i <= Math.max(source.x, dest.x) - 1; i++) {
            for (int j = Math.min(source.y, dest.y) + 1; j <= Math.max(source.y, dest.y) - 1; j++) {
                //if (showErrors && (distPointToLine(sourceX, sourceY, destX, destY, i, j) <= 0.5)) {
                //simulator.ExplorationImage.addErrorMarker(new Point(i, j), null, false);
                //}
                if (allowUnknown) {
                    if ((distPointToLine(source, dest, new Point(i, j)) <= 0.5) && obstacleAt(i, j)) {
                        if (showErrors) {
                            simulator.ExplorationImage.addErrorMarker(new Point(i, j), "obstacle", true);
                        }
                        return false;
                    }
                } else if ((distPointToLine(source, dest, new Point(i, j)) <= 0.5) && !freeSpaceAt(i, j)) {
                    if (showErrors) {
                        simulator.ExplorationImage.addErrorMarker(new Point(i, j), "unfree", true);
                    }
                    return false;
                }
            }
        }

        return true;
    }

    public boolean isOnLine(Point endPoint1, Point endPoint2, Point checkPoint) {
        return (Line2D.ptSegDist(endPoint1.x, endPoint1.y, endPoint2.x, endPoint2.y, checkPoint.x, checkPoint.y) < 1.0);
    }

    /**
     * This function finds the shortest distance from P3 to the line between P1 and P2
     *
     * @param a Line Point A
     * @param b Line Point B
     * @param c Point
     * @return Distance from c to the line a-b
     */
    public double distPointToLine(Point a, Point b, Point c) {
        if (c == a || c == b) {
            return 0;
        }

        double dist = Math.sqrt(Math.pow(b.y - a.y, 2) + Math.pow(b.x - a.x, 2));
        double slope = ((c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)) / Math.pow(dist, 2);

        double xIntersection = a.x + slope * (b.x - a.x);
        double yIntersection = a.y + slope * (b.y - a.y);

        double shortestDist = Math.sqrt(Math.pow(c.x - xIntersection, 2) + Math.pow(c.y - yIntersection, 2));

        return shortestDist;
    }

    // Returns distance to nearest wall, up to a maximum distance
    @Override
    public boolean obstacleWithinDistance(int x, int y, int minDistance) {
        for (int i = x - minDistance; i <= x + minDistance; i++) {
            for (int j = y - minDistance; j <= y + minDistance; j++) {
                if (locationExists(i, j)
                        && new Point(x, y).distance(i, j) <= minDistance
                        && obstacleAt(i, j)) {
                    return true;
                }
            }
        }
        return false;
    }

    public int getEmptySpacesWithinSquare(int x, int y, int minDistance) {
        int count = 0;
        for (int i = x - minDistance; i <= x + minDistance; i++) {
            for (int j = y - minDistance; j <= y + minDistance; j++) {
                if (locationExists(i, j)
                        && /*new Point(x,y).distance(i,j) <= minDistance &&*/ emptyAt(i, j) && ((i != x) || (j != y))) {
                    count++;
                }
            }
        }
        return count;
    }

    public int getSafeSpacesWithinSquare(int x, int y, int minDistance) {
        int count = 0;
        for (int i = x - minDistance; i <= x + minDistance; i++) {
            for (int j = y - minDistance; j <= y + minDistance; j++) {
                if (locationExists(i, j)
                        && /*new Point(x,y).distance(i,j) < minDistance &&*/ safeSpaceAt(i, j) && ((i != x) || (j != y))) {
                    count++;
                }
            }
        }
        return count;
    }

    public int numObstaclesOnLine(int x1, int y1, int x2, int y2) {
        int counter = 0;
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int distance = (int) (Math.sqrt(Math.pow(y2 - y1, 2) + Math.pow(x2 - x1, 2)));
        int currX, currY;

        for (int i = 0; i <= distance; i++) {
            currX = x1 + (int) (Math.cos(angle) * i);
            currY = y1 + (int) (Math.sin(angle) * i);

            if (this.obstacleAt(currX, currY)) {
                counter++;
            }
        }

        return counter;
    }

    // Rewritten by Alec
    public int numPossibleObstaclesOnLine(int x1, int y1, int x2, int y2) {
        int counter = 0;
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int xInc = (int) Math.cos(angle);
        int yInc = (int) Math.sin(angle);

        int distance = (int) (Math.sqrt(Math.pow(y2 - y1, 2) + Math.pow(x2 - x1, 2)));
        int currX = x1;
        int currY = y1;
        boolean insideWall = false; //flag to make sure a thick wall counts as one obstacle

        //every meter of unknown space we assume there is one wall.
        int unknownSpaceCounter = 0;
        int unknownSpaceWallLimit = 13; //0.078m/px makes it 1 meter

        for (int i = 0; i <= distance; i++) {
            currX += xInc;
            currY += yInc;

            if(this.freeSpaceAt(currX, currY)){
               insideWall = false;
               unknownSpaceCounter = 0;
               continue;
            }

            if(this.obstacleAt(currX, currY) && !insideWall){
                counter++;
                insideWall = true;
                unknownSpaceCounter = 0;
                continue;
            }

            unknownSpaceCounter++;
            if(unknownSpaceCounter >= unknownSpaceWallLimit){
                unknownSpaceCounter = 0;
                counter++;
            }
            /* original code
            if (!this.freeSpaceAt(currX, currY)) {
                if (!this.obstacleAt(currX, currY)) {
                    unknownSpaceCounter++;
                    if (unknownSpaceCounter > unknownSpaceWallLimit) {
                        System.out.println("added wall due to unknown space");
                        counter++;
                        unknownSpaceCounter = 0;
                    }
                } else {
                    unknownSpaceCounter = 0;
                }
                if (!insideWall) {
                    counter++;
                    System.out.println("found wall");
                    insideWall = true;
                }
            } else {
                insideWall = false;
            }
             */
        }

        return counter;
    }

    private void setBit(int xCoord, int yCoord, OccGridBit bit, int value) {
        setBit(xCoord, yCoord, bit.ordinal(), value);
    }

    private void setBit(int xCoord, int yCoord, int bit, int value) {
        hashCode = 0; // something changed, so delete hashcode
        this.skeleton = null; // and cached stuff
        this.skeletonList.clear();
        int bitValue = grid[xCoord][yCoord] & (byte) (Math.pow(2, bit));
        if (bitValue == 0) {
            if (value == 0) {
                return;
            } else {
                grid[xCoord][yCoord] += (byte) (Math.pow(2, bit));
            }
        } else if (value == 1) {
            return;
        } else {
            grid[xCoord][yCoord] -= (byte) (Math.pow(2, bit));
        }
    }

    private void setByte(int x, int y, byte value) {
        hashCode = 0; // something changed, so delete hashcode
        grid[x][y] = value;
    }

    @Override
    public int[][] getIntGrid() {
        int[][] intGrid = new int[width][height];
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (freeSpaceAt(i, j)) {
                    intGrid[j][i] = 1;
                } else {
                    intGrid[j][i] = 0;
                }
            }
        }
        return intGrid;
    }

    public int[][] getSkeleton() {
        if (this.skeleton == null) {
            this.skeleton = Skeleton.findSkeleton(this, false);
        }
        return this.skeleton;
    }

    public List<Point> getSkeletonList() {
        if (this.skeletonList.isEmpty()) {
            this.skeletonList = Skeleton.gridToList(getSkeleton());
        }
        return this.skeletonList;

    }
}
