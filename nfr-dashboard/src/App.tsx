import { Tab, Tabs } from '@mui/material';
import './App.css';
import { useState } from 'react';
import { useEntry } from '@frc-web-components/react';
import Teleop from './teleop/Teleop';
import Auto from './auto/Auto';
import Settings from './settings/Settings';
import ConnectionStatus from './components/ConnectionStatus';
import DisconnectedNotice from './components/DisconnectedFromServerNotice';
import PitMenu from './components/PitMenu';

function TabPanel(props: { children?: React.ReactNode, selected: number, index: number }) {
    return <div hidden={props.selected !== props.index}>
        {props.children}
    </div>
}

function App() {
    let [selected, setSelected] = useState(0);
    const [robotConnected, setRobotConnected] = useState(false);
    let [tabEntry, _setSelectedTab] = useEntry('/FWC/selectedTab', 0)
    const handleTabChange = (_event: React.ChangeEvent<{}>, newValue: number) => {
        if (tabsLocked) return;
        setSelected(newValue);
    }
    let [tabsLocked] = useState(false);
    let [hasCoral] = useEntry('/FWC/HasCoral', false);
    let color = hasCoral ? "#026afa" : "";
    const [selectedMenu, setSelectedMenu] = useState("");
    return (
        <>
            <div className="main-container" style={{ backgroundColor: color }}>
                <div className="header">
                    <Tabs id="header-tabs" value={tabsLocked ? tabEntry : selected} onChange={handleTabChange}>
                        <Tab label="Teleop" />
                        <Tab label="Autonomous" />
                        <Tab label="Settings" />
                    </Tabs>
                    <ConnectionStatus sourceKey="/FWC/ConnectionStatus" changeConnectionStatus={setRobotConnected} />
                </div>
                <div>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={0}>
                        <Teleop />
                    </TabPanel>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={1}>
                        <Auto />
                    </TabPanel>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={2}>
                        <PitMenu menuToDisplay={[
                            {
                                key: "settings",
                                label: "Settings",
                                value: {
                                    node: <Settings />,
                                    children: [
                                        {
                                            key: "robot-settings",
                                            label: "Robot Settings",
                                            value: <div>Robot Settings Content</div>
                                        },
                                        {
                                            key: "driver-settings",
                                            label: "Driver Settings",
                                            value: <div>Driver Settings Content</div>
                                        }
                                    ]
                                }
                            },
                            {
                                key: "help",
                                label: "Help",
                                value: <div>Help Content</div>
                            }
                        ]} selectedMenu={selectedMenu} setSelectedMenu={setSelectedMenu} />
                    </TabPanel>
                </div>
                <DisconnectedNotice isDisconnected={!robotConnected} />
            </div>
        </>
    );
}

export default App;
