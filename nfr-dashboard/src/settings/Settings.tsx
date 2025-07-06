import { Accordion, AccordionDetails, AccordionSummary } from '@mui/material';
import DriveWidget from './DriveWidget';
import './Settings.css';
import PoseWidget from './PoseWidget';
import CameraDetails from './CameraDetails';
import ElevatorWidget from './ElevatorWidget';
function Settings() {
    let ip = new URLSearchParams(window.location.search).get("ip") || "10.1.72.2";
    let ipBase = ip.split('.').slice(0, 3).join('.');
    let frontLeftCameraIp = `${ipBase}.11`;
    let frontRightCameraIp = `${ipBase}.12`;
    let backLeftCameraIp = `${ipBase}.13`;
    let backRightCameraIp = `${ipBase}.14`;
    return (
        <div className="settings-container">
            <Accordion>
                <AccordionSummary>Drive</AccordionSummary>
                <AccordionDetails><DriveWidget /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Pose</AccordionSummary>
                <AccordionDetails><PoseWidget /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Inner Elevator</AccordionSummary>
                <AccordionDetails><ElevatorWidget name="InnerElevator" /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Outer Elevator</AccordionSummary>
                <AccordionDetails><ElevatorWidget name="OuterElevator" /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Front Left Camera</AccordionSummary>
                <AccordionDetails><CameraDetails ip={frontLeftCameraIp} /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Front Right Camera</AccordionSummary>
                <AccordionDetails><CameraDetails ip={frontRightCameraIp} /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Back Left Camera</AccordionSummary>
                <AccordionDetails><CameraDetails ip={backLeftCameraIp} /></AccordionDetails>
            </Accordion>
            <Accordion>
                <AccordionSummary>Back Right Camera</AccordionSummary>
                <AccordionDetails><CameraDetails ip={backRightCameraIp} /></AccordionDetails>
            </Accordion>
        </div>
    );
}
export default Settings;