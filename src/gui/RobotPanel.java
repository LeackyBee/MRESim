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
 *     the Free Software Foundation, either version 3 of the License, or
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
package gui;

import config.RobotConfig;
import javax.swing.JLabel;

/**
 *
 * @author julh
 */
public class RobotPanel extends javax.swing.JPanel {

    private MainGUI mainGUI;

    /**
     * Creates new form RobotPanel
     *
     * @param main
     * @param currRobot
     */
    public RobotPanel(MainGUI main, RobotConfig currRobot) {
        mainGUI = main;

        initComponents();
        labelRank.setText(String.valueOf(currRobot.getRobotNumber()));
        labelName.setText(currRobot.getName());
    }

    public JLabel getLabelRole() {
        return labelRole;
    }

    public JLabel getLabelState() {
        return labelState;
    }

    public JLabel getLabelPower() {
        return labelPower;
    }

    public void setShowAgent(boolean val) {
        toggleAgent.setSelected(val);
        mainGUI.updateShowSettingsAgents();
    }

    public void setShowFreeSpace(boolean val) {
        toggleFreeSpace.setSelected(val);
        mainGUI.updateShowSettingsAgents();
    }

    public void setShowCommRange(boolean val) {
        toggleCommRange.setSelected(val);
        mainGUI.updateShowSettingsAgents();
    }

    public void setShowRendezvous(boolean val) {
        toggleRendezvous.setSelected(val);
        mainGUI.updateShowSettingsAgents();
    }

    public void setShowSkeleton(boolean val) {
        toggleSkeleton.setSelected(val);
        mainGUI.updateShowSettingsAgents();
    }

    public boolean showAgent() {
        return toggleAgent.isSelected();
    }

    public boolean showFreeSpace() {
        return toggleFreeSpace.isSelected();
    }

    public boolean showSafeSpace() {
        return toggleSafeSpace.isSelected();
    }

    public boolean showCommRange() {
        return toggleCommRange.isSelected();
    }

    public boolean showFrontiers() {
        return toggleFrontiers.isSelected();
    }

    public boolean showPath() {
        return togglePath.isSelected();
    }

    public boolean showSkeleton() {
        return toggleSkeleton.isSelected();
    }

    public boolean showRendezvous() {
        return toggleRendezvous.isSelected();
    }

    public boolean showBorderSkel() {
        return toggleBorderSkeleton.isSelected();
    }

    public boolean showRVWalls() {
        return toggleRVWalls.isSelected();
    }

    public boolean loggingState() {
        return toggleLoggingState.isSelected();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        panelRobotConfig = new javax.swing.JPanel();
        labelName = new javax.swing.JLabel();
        labelRank = new javax.swing.JLabel();
        labelRole = new javax.swing.JLabel();
        labelState = new javax.swing.JLabel();
        labelPower = new javax.swing.JLabel();
        jToolBar1 = new javax.swing.JToolBar();
        toggleAgent = new javax.swing.JToggleButton();
        togglePath = new javax.swing.JToggleButton();
        toggleFreeSpace = new javax.swing.JToggleButton();
        toggleSafeSpace = new javax.swing.JToggleButton();
        toggleFrontiers = new javax.swing.JToggleButton();
        toggleCommRange = new javax.swing.JToggleButton();
        toggleSkeleton = new javax.swing.JToggleButton();
        toggleRendezvous = new javax.swing.JToggleButton();
        toggleBorderSkeleton = new javax.swing.JToggleButton();
        toggleRVWalls = new javax.swing.JToggleButton();
        jSeparator1 = new javax.swing.JSeparator();
        toggleLoggingState = new javax.swing.JToggleButton();

        setBorder(javax.swing.BorderFactory.createEmptyBorder(1, 1, 1, 1));
        setMaximumSize(new java.awt.Dimension(275, 80));
        setMinimumSize(new java.awt.Dimension(275, 0));
        setName(""); // NOI18N
        setPreferredSize(new java.awt.Dimension(275, 80));

        panelRobotConfig.setName("panelRobotConfig"); // NOI18N

        labelName.setFont(new java.awt.Font("Arial", 1, 11)); // NOI18N
        labelName.setHorizontalAlignment(javax.swing.SwingConstants.LEFT);
        labelName.setText("ComStation");

        labelRank.setFont(new java.awt.Font("Arial", 1, 11)); // NOI18N
        labelRank.setHorizontalAlignment(javax.swing.SwingConstants.LEFT);
        labelRank.setText("1.");

        labelRole.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelRole.setHorizontalAlignment(javax.swing.SwingConstants.LEFT);
        labelRole.setText("Role");

        labelState.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelState.setText("State");

        labelPower.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelPower.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelPower.setText("0");

        javax.swing.GroupLayout panelRobotConfigLayout = new javax.swing.GroupLayout(panelRobotConfig);
        panelRobotConfig.setLayout(panelRobotConfigLayout);
        panelRobotConfigLayout.setHorizontalGroup(
            panelRobotConfigLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panelRobotConfigLayout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addGroup(panelRobotConfigLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(panelRobotConfigLayout.createSequentialGroup()
                        .addComponent(labelRank, javax.swing.GroupLayout.PREFERRED_SIZE, 22, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(labelName, javax.swing.GroupLayout.PREFERRED_SIZE, 84, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(labelPower, javax.swing.GroupLayout.PREFERRED_SIZE, 43, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(panelRobotConfigLayout.createSequentialGroup()
                        .addComponent(labelRole, javax.swing.GroupLayout.PREFERRED_SIZE, 92, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(labelState, javax.swing.GroupLayout.PREFERRED_SIZE, 105, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addGap(20, 20, 20))
        );
        panelRobotConfigLayout.setVerticalGroup(
            panelRobotConfigLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(panelRobotConfigLayout.createSequentialGroup()
                .addGroup(panelRobotConfigLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(labelRank, javax.swing.GroupLayout.DEFAULT_SIZE, 15, Short.MAX_VALUE)
                    .addComponent(labelName)
                    .addComponent(labelPower))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(panelRobotConfigLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(labelRole, javax.swing.GroupLayout.PREFERRED_SIZE, 14, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(labelState)))
        );

        jToolBar1.setBorder(javax.swing.BorderFactory.createEmptyBorder(1, 1, 1, 1));
        jToolBar1.setRollover(true);

        toggleAgent.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleAgent.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/agentsOn.png"))); // NOI18N
        toggleAgent.setSelected(true);
        toggleAgent.setToolTipText("Agent");
        toggleAgent.setBorderPainted(false);
        toggleAgent.setContentAreaFilled(false);
        toggleAgent.setFocusable(false);
        toggleAgent.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleAgent.setIconTextGap(0);
        toggleAgent.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleAgent.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleAgent.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleAgent.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleAgent.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleAgentActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleAgent);

        togglePath.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        togglePath.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/pathOn.png"))); // NOI18N
        togglePath.setSelected(true);
        togglePath.setToolTipText("Path");
        togglePath.setBorderPainted(false);
        togglePath.setContentAreaFilled(false);
        togglePath.setFocusable(false);
        togglePath.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        togglePath.setIconTextGap(0);
        togglePath.setMaximumSize(new java.awt.Dimension(25, 25));
        togglePath.setMinimumSize(new java.awt.Dimension(25, 25));
        togglePath.setPreferredSize(new java.awt.Dimension(25, 25));
        togglePath.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        togglePath.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                togglePathActionPerformed(evt);
            }
        });
        jToolBar1.add(togglePath);

        toggleFreeSpace.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleFreeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/freespaceOn.png"))); // NOI18N
        toggleFreeSpace.setSelected(true);
        toggleFreeSpace.setToolTipText("Free Space");
        toggleFreeSpace.setBorderPainted(false);
        toggleFreeSpace.setContentAreaFilled(false);
        toggleFreeSpace.setFocusable(false);
        toggleFreeSpace.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleFreeSpace.setIconTextGap(0);
        toggleFreeSpace.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleFreeSpace.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleFreeSpace.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleFreeSpace.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleFreeSpace.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleFreeSpace.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleFreeSpaceActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleFreeSpace);

        toggleSafeSpace.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleSafeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/safespaceOff.png"))); // NOI18N
        toggleSafeSpace.setToolTipText("Safe Space");
        toggleSafeSpace.setBorderPainted(false);
        toggleSafeSpace.setContentAreaFilled(false);
        toggleSafeSpace.setFocusable(false);
        toggleSafeSpace.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleSafeSpace.setIconTextGap(0);
        toggleSafeSpace.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleSafeSpace.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleSafeSpace.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleSafeSpace.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleSafeSpace.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleSafeSpace.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleSafeSpaceActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleSafeSpace);

        toggleFrontiers.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleFrontiers.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/frontiersOn.png"))); // NOI18N
        toggleFrontiers.setSelected(true);
        toggleFrontiers.setToolTipText("Frontiers");
        toggleFrontiers.setBorderPainted(false);
        toggleFrontiers.setContentAreaFilled(false);
        toggleFrontiers.setFocusable(false);
        toggleFrontiers.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleFrontiers.setIconTextGap(0);
        toggleFrontiers.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleFrontiers.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleFrontiers.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleFrontiers.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleFrontiers.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleFrontiers.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleFrontiersActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleFrontiers);

        toggleCommRange.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleCommRange.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/commrangeOn.png"))); // NOI18N
        toggleCommRange.setSelected(true);
        toggleCommRange.setToolTipText("Communication Range");
        toggleCommRange.setBorderPainted(false);
        toggleCommRange.setContentAreaFilled(false);
        toggleCommRange.setFocusable(false);
        toggleCommRange.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleCommRange.setIconTextGap(0);
        toggleCommRange.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleCommRange.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleCommRange.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleCommRange.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleCommRange.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleCommRange.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleCommRangeActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleCommRange);

        toggleSkeleton.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/skeletonOn.png"))); // NOI18N
        toggleSkeleton.setSelected(true);
        toggleSkeleton.setToolTipText("Show information on how the RV location was selected");
        toggleSkeleton.setBorderPainted(false);
        toggleSkeleton.setContentAreaFilled(false);
        toggleSkeleton.setFocusable(false);
        toggleSkeleton.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleSkeleton.setIconTextGap(0);
        toggleSkeleton.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleSkeleton.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleSkeleton.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleSkeleton.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleSkeleton.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleSkeleton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleSkeletonActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleSkeleton);

        toggleRendezvous.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleRendezvous.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/rvpointOff.png"))); // NOI18N
        toggleRendezvous.setToolTipText("Rendezvous Points");
        toggleRendezvous.setBorderPainted(false);
        toggleRendezvous.setContentAreaFilled(false);
        toggleRendezvous.setFocusable(false);
        toggleRendezvous.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleRendezvous.setIconTextGap(0);
        toggleRendezvous.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleRendezvous.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleRendezvous.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleRendezvous.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleRendezvous.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleRendezvous.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleRendezvousActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleRendezvous);

        toggleBorderSkeleton.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleBorderSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderSkelOff.png"))); // NOI18N
        toggleBorderSkeleton.setToolTipText("Sampled points for comm through obstacles");
        toggleBorderSkeleton.setBorderPainted(false);
        toggleBorderSkeleton.setContentAreaFilled(false);
        toggleBorderSkeleton.setFocusable(false);
        toggleBorderSkeleton.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleBorderSkeleton.setIconTextGap(0);
        toggleBorderSkeleton.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleBorderSkeleton.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleBorderSkeleton.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleBorderSkeleton.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleBorderSkeleton.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleBorderSkeleton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleBorderSkeletonActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleBorderSkeleton);

        toggleRVWalls.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        toggleRVWalls.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderRVOn.png"))); // NOI18N
        toggleRVWalls.setSelected(true);
        toggleRVWalls.setToolTipText("Potentiall RV points for comm through obstacles");
        toggleRVWalls.setBorderPainted(false);
        toggleRVWalls.setContentAreaFilled(false);
        toggleRVWalls.setFocusable(false);
        toggleRVWalls.setHorizontalTextPosition(javax.swing.SwingConstants.CENTER);
        toggleRVWalls.setIconTextGap(0);
        toggleRVWalls.setMargin(new java.awt.Insets(2, 0, 2, 1));
        toggleRVWalls.setMaximumSize(new java.awt.Dimension(25, 25));
        toggleRVWalls.setMinimumSize(new java.awt.Dimension(25, 25));
        toggleRVWalls.setPreferredSize(new java.awt.Dimension(25, 25));
        toggleRVWalls.setVerticalTextPosition(javax.swing.SwingConstants.BOTTOM);
        toggleRVWalls.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleRVWallsActionPerformed(evt);
            }
        });
        jToolBar1.add(toggleRVWalls);

        toggleLoggingState.setText("Log");
        toggleLoggingState.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleLoggingStateActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jSeparator1, javax.swing.GroupLayout.Alignment.TRAILING)
            .addGroup(layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                    .addComponent(jToolBar1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(panelRobotConfig, javax.swing.GroupLayout.PREFERRED_SIZE, 208, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(toggleLoggingState)
                        .addGap(16, 16, 16)))
                .addContainerGap(5, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(panelRobotConfig, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(toggleLoggingState)
                        .addGap(0, 0, Short.MAX_VALUE)))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jToolBar1, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jSeparator1, javax.swing.GroupLayout.PREFERRED_SIZE, 4, Short.MAX_VALUE))
        );
    }// </editor-fold>//GEN-END:initComponents

    private void redrawImage() {
        try {
            mainGUI.getExploration().updateImage(true);
        } catch (NullPointerException e) {
            System.err.println("Error: toggle button pressed, no image to update.");
        }
    }
private void toggleAgentActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleAgentActionPerformed
    if (toggleAgent.isSelected()) {
        toggleAgent.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/agentsOn.png")));
    } else {
        toggleAgent.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/agentsOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleAgentActionPerformed

private void toggleCommRangeActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleCommRangeActionPerformed
    if (toggleCommRange.isSelected()) {
        toggleCommRange.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/commrangeOn.png")));
    } else {
        toggleCommRange.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/commrangeOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleCommRangeActionPerformed

private void toggleFreeSpaceActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleFreeSpaceActionPerformed
    if (toggleFreeSpace.isSelected()) {
        toggleFreeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/freespaceOn.png")));
    } else {
        toggleFreeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/freespaceOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleFreeSpaceActionPerformed

private void toggleSafeSpaceActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleSafeSpaceActionPerformed
    if (toggleSafeSpace.isSelected()) {
        toggleSafeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/safespaceOn.png")));
    } else {
        toggleSafeSpace.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/safespaceOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleSafeSpaceActionPerformed

private void toggleFrontiersActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleFrontiersActionPerformed
    if (toggleFrontiers.isSelected()) {
        toggleFrontiers.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/frontiersOn.png")));
    } else {
        toggleFrontiers.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/frontiersOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleFrontiersActionPerformed

private void togglePathActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_togglePathActionPerformed
    if (togglePath.isSelected()) {
        togglePath.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/pathOn.png")));
    } else {
        togglePath.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/pathOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_togglePathActionPerformed

private void toggleSkeletonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleSkeletonActionPerformed
    if (toggleSkeleton.isSelected()) {
        toggleSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/skeletonOn.png")));
    } else {
        toggleSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/skeletonOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleSkeletonActionPerformed

private void toggleRendezvousActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleRendezvousActionPerformed
    if (toggleRendezvous.isSelected()) {
        toggleRendezvous.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/rvpointOn.png")));
    } else {
        toggleRendezvous.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/rvpointOff.png")));
    }
    redrawImage();
    mainGUI.updateShowSettingsAgents();
}//GEN-LAST:event_toggleRendezvousActionPerformed

    private void toggleBorderSkeletonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleBorderSkeletonActionPerformed
        if (toggleBorderSkeleton.isSelected()) {
            toggleBorderSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderSkelOn.png")));
        } else {
            toggleBorderSkeleton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderSkelOff.png")));
        }
        redrawImage();
        mainGUI.updateShowSettingsAgents();
    }//GEN-LAST:event_toggleBorderSkeletonActionPerformed

    private void toggleRVWallsActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleRVWallsActionPerformed
        if (toggleRVWalls.isSelected()) {
            toggleRVWalls.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderRVOn.png")));
        } else {
            toggleRVWalls.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/borderRVOff.png")));
        }
        redrawImage();
        mainGUI.updateShowSettingsAgents();
    }//GEN-LAST:event_toggleRVWallsActionPerformed

    private void toggleLoggingStateActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleLoggingStateActionPerformed
        mainGUI.updateShowSettingsAgents();
    }//GEN-LAST:event_toggleLoggingStateActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JSeparator jSeparator1;
    private javax.swing.JToolBar jToolBar1;
    private javax.swing.JLabel labelName;
    private javax.swing.JLabel labelPower;
    private javax.swing.JLabel labelRank;
    private javax.swing.JLabel labelRole;
    private javax.swing.JLabel labelState;
    private javax.swing.JPanel panelRobotConfig;
    private javax.swing.JToggleButton toggleAgent;
    private javax.swing.JToggleButton toggleBorderSkeleton;
    private javax.swing.JToggleButton toggleCommRange;
    private javax.swing.JToggleButton toggleFreeSpace;
    private javax.swing.JToggleButton toggleFrontiers;
    private javax.swing.JToggleButton toggleLoggingState;
    private javax.swing.JToggleButton togglePath;
    private javax.swing.JToggleButton toggleRVWalls;
    private javax.swing.JToggleButton toggleRendezvous;
    private javax.swing.JToggleButton toggleSafeSpace;
    private javax.swing.JToggleButton toggleSkeleton;
    // End of variables declaration//GEN-END:variables

}
