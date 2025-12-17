import React from 'react';
import { ChatWidget } from '../components/ChatWidget';

interface RootProps {
  children: React.ReactNode;
}

// This wrapper adds the ChatWidget to all pages
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
