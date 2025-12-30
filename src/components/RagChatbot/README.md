# RAG Chatbot Component

A React component that integrates a RAG (Retrieval-Augmented Generation) chatbot into the Docusaurus documentation site.

## Features

- **Full Question Mode**: Ask questions about the entire documentation corpus
- **Selected Text Mode**: Ask questions specifically about selected text on the page
- **Loading States**: Visual indicators during API requests
- **Error Handling**: Graceful error display and recovery
- **Environment Configuration**: Supports different backend URLs for local/development/deployed environments

## Usage

### Basic Integration

```tsx
import RagChatbot from './components/RagChatbot/RagChatbot';

function MyPage() {
  return (
    <div>
      <h1>My Documentation Page</h1>
      <p>Content of my page...</p>

      <RagChatbot />
    </div>
  );
}
```

### With Custom Backend URL

```tsx
<RagChatbot backendUrl="https://your-deployed-backend.com" />
```

## Environment Configuration

The component supports the following environment variables:

- `REACT_APP_RAG_BACKEND_URL`: The URL of the RAG backend service
  - Example: `REACT_APP_RAG_BACKEND_URL=http://localhost:8000`

## API Integration

The component communicates with the FastAPI backend using the following endpoint:
- POST `/chat` - Sends user queries and receives AI responses

## Modes

### Full Question Mode
- Default mode
- Searches entire documentation corpus
- User types any question about the documentation

### Selected Text Mode
- Activate by clicking "Select Text" button
- First select text on the page by highlighting it
- Then ask questions specifically about the selected text
- Useful for clarifying specific sections

## Styling

The component includes its own CSS file (`RagChatbot.css`) with responsive design that works on both desktop and mobile devices.

## Error Handling

- Network errors are caught and displayed to the user
- Backend errors are handled gracefully
- Loading states prevent duplicate submissions
- Empty states guide new users

## Development

To run with a local backend:

1. Start the FastAPI backend:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

2. Set the environment variable in `.env.local`:
   ```
   REACT_APP_RAG_BACKEND_URL=http://localhost:8000
   ```

3. Start the Docusaurus development server:
   ```bash
   npm run start
   ```

## Deployment

For deployment to GitHub Pages or other platforms:

1. Update the backend URL to point to your deployed FastAPI service
2. Ensure CORS is configured properly on the backend
3. Test the integration in the deployed environment