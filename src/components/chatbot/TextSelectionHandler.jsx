import React, { useEffect, useState } from 'react';
import { MessageSquare } from 'lucide-react';

/**
 * TextSelectionHandler component
 * Displays a floating button when text is selected, allowing users to ask questions about the selection
 */
const TextSelectionHandler = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 10
        });
        setIsVisible(true);
      } else {
        setIsVisible(false);
      }
    };

    const handleClickAway = (e) => {
      // Hide the button if clicking outside of it
      if (!e.target.closest('.text-selection-button')) {
        setIsVisible(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('mousedown', handleClickAway);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('mousedown', handleClickAway);
    };
  }, []);

  const handleAskAboutSelection = () => {
    if (onTextSelected && selectedText) {
      onTextSelected(selectedText);
      setIsVisible(false);
      window.getSelection().removeAllRanges();
    }
  };

  if (!isVisible) return null;

  return (
    <button
      className="text-selection-button fixed z-50 bg-gradient-to-r from-green-600 to-teal-500 text-white px-4 py-2 rounded-lg shadow-lg hover:shadow-xl transition-all transform hover:scale-105 flex items-center gap-2 animate-fadeIn"
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        transform: 'translateX(-50%) translateY(-100%)',
      }}
      onClick={handleAskAboutSelection}
    >
      <MessageSquare className="w-4 h-4" />
      <span className="text-sm font-medium">Ask AI about this</span>
    </button>
  );
};

export default TextSelectionHandler;
