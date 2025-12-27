import React from 'react';
import HumanoidChatbot from '../components/chatbot/chatbot';

export default function Root({ children }:{children: React.ReactNode}) {
  return (
    <>
      {children}
      <HumanoidChatbot />
    </>
  );
}
