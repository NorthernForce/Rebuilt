import { Tab, Tabs } from '@mui/material';
import './App.css';
import { useState } from 'react';
import { useEntry } from '@frc-web-components/react';
import Teleop from './teleop/Teleop';
import Auto from './auto/Auto';
import ConnectionStatus from './components/ConnectionStatus';
import DisconnectedNotice from './components/DisconnectedFromServerNotice';
import NavigatableTreeDisplay from './components/NavigatableTreeDisplay';
import MotorChip from './components/MotorChip';
import ArmControl from './components/ArmControl';

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
                <div style={{height: '100%'}}>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={0}>
                        <Teleop />
                    </TabPanel>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={1}>
                        <Auto />
                    </TabPanel>
                    <TabPanel selected={tabsLocked ? tabEntry : selected} index={2}>
                        <div style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
                            <NavigatableTreeDisplay items={[
                                {
                                    id: 'subsystems',
                                    label: 'Subsystems',
                                    children: []
                                },
                                {
                                    id: 'motors',
                                    label: 'Motors',
                                    children: Array.from({ length: 10 }, (_, i) => ({
                                        id: `motor-${i + 1}`,
                                        label: `Motor ${i + 1}`,
                                        display: <MotorChip motorId={i + 1} />
                                    }))
                                }
                            ]} />
                        </div>
                    </TabPanel>
                </div>
                <DisconnectedNotice isDisconnected={!robotConnected} />
            </div>
        </>
    );
}

export default App;
