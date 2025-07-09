import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
import { NT4Provider } from '@frc-web-components/react';

let url = new URL(window.location.href);
let ip = url.searchParams.get('ip') || url.hostname || 'localhost';

createRoot(document.getElementById('app')!).render(
  <StrictMode>
    <NT4Provider address={ip}>
      <App />
    </NT4Provider>
  </StrictMode>
)