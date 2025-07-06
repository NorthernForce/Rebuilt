import { Canvas, CanvasMjpgStream } from '@frc-web-components/react';
import './CameraDetails.css';
function CameraDetails(props: { ip: string })
{
    return (
        <div className="camera-details-container">
            <Canvas className="details-feed">
                <CanvasMjpgStream origin={[0,0]} crosshairColor="white" srcs={[`http://${props.ip}/stream.mjpg`]} />
            </Canvas>
        </div>
    );
}
export default CameraDetails;