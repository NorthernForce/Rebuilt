import { Button, Dialog, DialogActions, DialogContent, DialogTitle } from "@mui/material";
import { useState } from "react";

interface DisconnectedNoticeProps {
    isDisconnected?: boolean;
}

function DisconnectedNotice({ isDisconnected }: DisconnectedNoticeProps) {
    const [isIgnored, setIsIgnored] = useState(false);
    return (
        <Dialog open={(isDisconnected ?? true) && !isIgnored} onClose={() => setIsIgnored(true)}>
            <DialogTitle>Disconnected from Server</DialogTitle>
            <DialogContent>
                <p>You are currently disconnected from the server.</p>
            </DialogContent>
            <DialogActions>
                {/* Try connecting to the robot over Wi-Fi (for competitions) */}
                <Button
                    variant="outlined"
                    color="info"
                    onClick={() => {
                        // Add (or change) the URL search params to be `?ip=10.1.72.2`
                        window.location.href = "http://10.1.72.2:5800?ip=10.1.72.2";
                    }}
                >
                    Connect to Robot (Wi-Fi)
                </Button>
                {/* Try connecting to the robot over USB (for practice) */}
                <Button
                    variant="outlined"
                    color="info"
                    onClick={() => {
                        // Add (or change) the URL search params to be `?ip=172.22.11.2`
                        window.location.href = "http://172.22.11.2:5800?ip=172.22.11.2";
                    }}
                >
                    Connect to Robot (USB)
                </Button>
                {/* Connect to development server */}
                <Button
                    variant="outlined"
                    color="info"
                    onClick={() => {
                        // Add (or change) the URL search params to be `?ip=localhost`
                        window.location.href = "http://localhost:5173?ip=localhost";
                    }}
                >
                    Connect to Dev Server
                </Button>
                <Button
                    variant="outlined"
                    color="secondary"
                    onClick={() => {
                        setIsIgnored(true);
                    }}
                >
                    Close
                </Button>
            </DialogActions>
        </Dialog>
    );
}

export default DisconnectedNotice;