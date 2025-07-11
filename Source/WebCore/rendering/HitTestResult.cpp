/*
 * Copyright (C) 2006-2024 Apple Inc. All rights reserved.
 * Copyright (C) 2014 Google Inc. All rights reserved.
 * Copyright (C) 2012 Nokia Corporation and/or its subsidiary(-ies)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
*/

#include "config.h"
#include "HitTestResult.h"

#include "CachedImage.h"
#include "ContainerNodeInlines.h"
#include "DocumentMarkerController.h"
#include "Editor.h"
#include "ElementInlines.h"
#include "File.h"
#include "FrameSelection.h"
#include "HTMLAnchorElement.h"
#include "HTMLAttachmentElement.h"
#include "HTMLEmbedElement.h"
#include "HTMLImageElement.h"
#include "HTMLInputElement.h"
#include "HTMLObjectElement.h"
#include "HTMLTextAreaElement.h"
#include "HTMLVideoElement.h"
#include "ImageOverlay.h"
#include "LocalFrame.h"
#include "NodeInlines.h"
#include "OriginAccessPatterns.h"
#include "Page.h"
#include "PseudoElement.h"
#include "Range.h"
#include "RenderBlockFlow.h"
#include "RenderImage.h"
#include "RenderInline.h"
#include "SVGAElement.h"
#include "SVGElementTypeHelpers.h"
#include "SVGImageElement.h"
#include "Scrollbar.h"
#include "ShadowRoot.h"
#include "TextIterator.h"
#include "UserGestureIndicator.h"
#include "VisibleUnits.h"
#include "XLinkNames.h"
#include <wtf/TZoneMallocInlines.h>

#if ENABLE(SERVICE_CONTROLS)
#include "ImageControlsMac.h"
#endif

namespace WebCore {

using namespace HTMLNames;

WTF_MAKE_TZONE_ALLOCATED_IMPL(HitTestResult);

static inline void appendToNodeSet(const HitTestResult::NodeSet& source, HitTestResult::NodeSet& destination)
{
    for (auto& node : source)
        destination.add(node.copyRef());
}

HitTestResult::HitTestResult() = default;

HitTestResult::HitTestResult(const LayoutPoint& point)
    : m_hitTestLocation(point)
    , m_pointInInnerNodeFrame(point)
{
}

HitTestResult::HitTestResult(const LayoutRect& rect)
    : m_hitTestLocation { rect }
    , m_pointInInnerNodeFrame { rect.center() }
{
}

HitTestResult::HitTestResult(const HitTestLocation& other)
    : m_hitTestLocation(other)
    , m_pointInInnerNodeFrame(m_hitTestLocation.point())
{
}

HitTestResult::HitTestResult(const HitTestResult& other)
    : m_hitTestLocation(other.m_hitTestLocation)
    , m_innerNode(other.innerNode())
    , m_innerNonSharedNode(other.innerNonSharedNode())
    , m_pointInInnerNodeFrame(other.m_pointInInnerNodeFrame)
    , m_localPoint(other.localPoint())
    , m_innerURLElement(other.URLElement())
    , m_scrollbar(other.scrollbar())
    , m_isOverWidget(other.isOverWidget())
{
    // Only copy the NodeSet in case of list hit test.
    if (other.m_listBasedTestResult) {
        m_listBasedTestResult = makeUnique<NodeSet>();
        appendToNodeSet(*other.m_listBasedTestResult, *m_listBasedTestResult);
    }
}

HitTestResult::~HitTestResult() = default;

HitTestResult& HitTestResult::operator=(const HitTestResult& other)
{
    m_hitTestLocation = other.m_hitTestLocation;
    m_innerNode = other.innerNode();
    m_innerNonSharedNode = other.innerNonSharedNode();
    m_pointInInnerNodeFrame = other.m_pointInInnerNodeFrame;
    m_localPoint = other.localPoint();
    m_innerURLElement = other.URLElement();
    m_scrollbar = other.scrollbar();
    m_isOverWidget = other.isOverWidget();

    // Only copy the NodeSet in case of list hit test.
    if (other.m_listBasedTestResult) {
        m_listBasedTestResult = makeUnique<NodeSet>();
        appendToNodeSet(*other.m_listBasedTestResult, *m_listBasedTestResult);
    }

    return *this;
}

static Node* moveOutOfUserAgentShadowTree(Node& node)
{
    if (node.isInShadowTree()) {
        if (ShadowRoot* root = node.containingShadowRoot()) {
            if (root->mode() == ShadowRootMode::UserAgent)
                return root->host();
        }
    }
    return &node;
}

void HitTestResult::setToNonUserAgentShadowAncestor()
{
    if (Node* node = innerNode()) {
        node = moveOutOfUserAgentShadowTree(*node);
        setInnerNode(node);
    }
    if (Node *node = innerNonSharedNode()) {
        node = moveOutOfUserAgentShadowTree(*node);
        setInnerNonSharedNode(node);
    }
}

void HitTestResult::setInnerNode(Node* node)
{
    auto* pseudoElement = dynamicDowncast<PseudoElement>(node);
    m_innerNode = pseudoElement ? pseudoElement->hostElement() : node;
}
    
void HitTestResult::setInnerNonSharedNode(Node* node)
{
    auto* pseudoElement = dynamicDowncast<PseudoElement>(node);
    m_innerNonSharedNode = pseudoElement ? pseudoElement->hostElement() : node;
}

void HitTestResult::setURLElement(Element* n) 
{ 
    m_innerURLElement = n; 
}

void HitTestResult::setScrollbar(RefPtr<Scrollbar>&& scrollbar)
{
    m_scrollbar = WTFMove(scrollbar);
}

LocalFrame* HitTestResult::innerNodeFrame() const
{
    if (m_innerNonSharedNode)
        return m_innerNonSharedNode->document().frame();
    if (m_innerNode)
        return m_innerNode->document().frame();
    return 0;
}

LocalFrame* HitTestResult::frame() const
{
    if (m_innerNonSharedNode)
        return m_innerNonSharedNode->document().frame();

    return nullptr;
}

LocalFrame* HitTestResult::targetFrame() const
{
    if (!m_innerURLElement)
        return nullptr;

    auto* frame = m_innerURLElement->document().frame();
    if (!frame)
        return nullptr;

    return dynamicDowncast<LocalFrame>(frame->tree().findBySpecifiedName(m_innerURLElement->target(), *frame));
}

bool HitTestResult::isSelected() const
{
    if (!m_innerNonSharedNode)
        return false;

    auto* frame = m_innerNonSharedNode->document().frame();
    if (!frame)
        return false;

    return frame->selection().contains(m_hitTestLocation.point());
}

bool HitTestResult::allowsFollowingLink() const
{
    auto linkURL = absoluteLinkURL();
    if (linkURL.isEmpty())
        return false;

    RefPtr innerFrame = innerNodeFrame();
    if (!innerFrame)
        return false;

    RefPtr document = innerFrame->document();
    if (!document)
        return false;

    return document->protectedSecurityOrigin()->canDisplay(linkURL, OriginAccessPatternsForWebProcess::singleton());
}

bool HitTestResult::allowsFollowingImageURL() const
{
    auto linkURL = absoluteImageURL();
    if (linkURL.isEmpty())
        return false;

    RefPtr innerFrame = innerNodeFrame();
    if (!innerFrame)
        return false;

    RefPtr document = innerFrame->document();
    if (!document)
        return false;

    return document->protectedSecurityOrigin()->canDisplay(linkURL, OriginAccessPatternsForWebProcess::singleton());
}

String HitTestResult::selectedText() const
{
    if (!m_innerNonSharedNode)
        return emptyString();

    auto* frame = m_innerNonSharedNode->document().frame();
    if (!frame)
        return emptyString();

    auto range = frame->selection().selection().toNormalizedRange();
    if (!range)
        return emptyString();

    // Look for a character that's not just a separator.
    for (TextIterator it(*range); !it.atEnd(); it.advance()) {
        int length = it.text().length();
        for (int i = 0; i < length; ++i) {
            if (!(U_GET_GC_MASK(it.text()[i]) & U_GC_Z_MASK))
                return frame->displayStringModifiedByEncoding(frame->editor().selectedText());
        }
    }
    return emptyString();
}

String HitTestResult::spellingToolTip(TextDirection& dir) const
{
    dir = TextDirection::LTR;
    // Return the tool tip string associated with this point, if any. Only markers associated with bad grammar
    // currently supply strings, but maybe someday markers associated with misspelled words will also.
    if (!m_innerNonSharedNode)
        return String();
    
    CheckedPtr markers = m_innerNonSharedNode->document().markersIfExists();
    if (!markers)
        return String();
    WeakPtr marker = markers->markerContainingPoint(m_hitTestLocation.point(), DocumentMarkerType::Grammar);
    if (!marker)
        return String();

    if (CheckedPtr renderer = m_innerNonSharedNode->renderer())
        dir = renderer->writingMode().computedTextDirection();
    return marker->description();
}

String HitTestResult::replacedString() const
{
    // Return the replaced string associated with this point, if any. This marker is created when a string is autocorrected, 
    // and is used for generating a contextual menu item that allows it to easily be changed back if desired.
    if (!m_innerNonSharedNode)
        return String();

    CheckedPtr markers = m_innerNonSharedNode->document().markersIfExists();
    if (!markers)
        return String();
    WeakPtr marker = markers->markerContainingPoint(m_hitTestLocation.point(), DocumentMarkerType::Replacement);
    if (!marker)
        return String();
    
    return marker->description();
}    
    
String HitTestResult::title(TextDirection& dir) const
{
    dir = TextDirection::LTR;
    // Find the title in the nearest enclosing DOM node.
    // For <area> tags in image maps, walk the tree for the <area>, not the <img> using it.
    for (Node* titleNode = m_innerNode.get(); titleNode; titleNode = titleNode->parentInComposedTree()) {
        if (RefPtr titleElement = dynamicDowncast<Element>(*titleNode)) {
            auto title = titleElement->title();
            if (!title.isNull()) {
                if (auto renderer = titleElement->renderer())
                    dir = renderer->writingMode().computedTextDirection();
                return title;
            }
        }
    }
    return String();
}

String HitTestResult::innerTextIfTruncated(TextDirection& dir) const
{
    for (auto* truncatedNode = m_innerNode.get(); truncatedNode; truncatedNode = truncatedNode->parentInComposedTree()) {
        auto* element = dynamicDowncast<Element>(*truncatedNode);
        if (!element)
            continue;

        if (auto* block = dynamicDowncast<RenderBlockFlow>(element->renderer())) {
            if (block->style().textOverflow() == TextOverflow::Ellipsis) {
                for (auto lineBox = InlineIterator::firstLineBoxFor(*block); lineBox; lineBox.traverseNext()) {
                    if (lineBox->hasEllipsis()) {
                        dir = block->writingMode().computedTextDirection();
                        return element->innerText();
                    }
                }
            }
            break;
        }
    }

    dir = TextDirection::LTR;
    return String();
}

String displayString(const String& string, const Node* node)
{
    if (!node)
        return string;
    return node->document().displayStringModifiedByEncoding(string);
}

String HitTestResult::altDisplayString() const
{
    if (!m_innerNonSharedNode)
        return String();

    if (RefPtr image = dynamicDowncast<HTMLImageElement>(*m_innerNonSharedNode))
        return displayString(image->attributeWithoutSynchronization(altAttr), m_innerNonSharedNode.get());

    if (RefPtr input = dynamicDowncast<HTMLInputElement>(*m_innerNonSharedNode))
        return displayString(input->attributeWithoutSynchronization(altAttr), m_innerNonSharedNode.get());

    return String();
}

RefPtr<Node> HitTestResult::nodeForImageData() const
{
    if (!m_innerNonSharedNode)
        return nullptr;

    if (ImageOverlay::isInsideOverlay(*m_innerNonSharedNode))
        return m_innerNonSharedNode->shadowHost();
    
#if ENABLE(SERVICE_CONTROLS)
    if (ImageControlsMac::isInsideImageControls(*m_innerNonSharedNode))
        return m_innerNonSharedNode->shadowHost();
#endif

    return m_innerNonSharedNode;
}

Image* HitTestResult::image() const
{
    auto imageNode = nodeForImageData();
    if (!imageNode)
        return nullptr;

    if (auto* image = dynamicDowncast<RenderImage>(imageNode->renderer())) {
        if (image->cachedImage() && !image->cachedImage()->errorOccurred())
            return image->cachedImage()->imageForRenderer(image);
    }

    return nullptr;
}

IntRect HitTestResult::imageRect() const
{
    if (!image())
        return IntRect();

    auto imageNode = nodeForImageData();
    if (!imageNode)
        return { };

    return imageNode->renderBox()->absoluteContentQuad().enclosingBoundingBox();
}

bool HitTestResult::hasEntireImage() const
{
    auto imageURL = absoluteImageURL();
    if (imageURL.isEmpty() || imageRect().isEmpty())
        return false;

    auto* innerFrame = innerNodeFrame();
    if (!innerFrame)
        return false;

    if (RefPtr page = innerFrame->page())
        return page->hasLocalDataForURL(imageURL);

    return false;
}

URL HitTestResult::absoluteImageURL() const
{
    auto imageNode = nodeForImageData();
    if (!imageNode)
        return { };

    auto renderer = imageNode->renderer();
    if (!renderer || !renderer->isImage())
        return { };

    if (RefPtr element = dynamicDowncast<Element>(*imageNode); element
        && (is<HTMLEmbedElement>(*element)
        || is<HTMLImageElement>(*element)
        || is<HTMLInputElement>(*element)
        || is<HTMLObjectElement>(*element)
        || is<SVGImageElement>(*element))) {
        auto imageURL = imageNode->document().completeURL(element->imageSourceURL());
        if (RefPtr page = imageNode->document().page())
            return page->applyLinkDecorationFiltering(imageURL, LinkDecorationFilteringTrigger::Unspecified);
        return imageURL;
    }

    return { };
}

URL HitTestResult::absolutePDFURL() const
{
    if (!m_innerNonSharedNode)
        return URL();

    RefPtr element = dynamicDowncast<HTMLPlugInImageElement>(*m_innerNonSharedNode);
    if (!element)
        return URL();

    auto url = m_innerNonSharedNode->document().completeURL(element->url());
    if (!url.isValid())
        return URL();

    if (element->serviceType() == "application/pdf"_s || (element->serviceType().isEmpty() && url.path().endsWithIgnoringASCIICase(".pdf"_s)))
        return url;
    return URL();
}

URL HitTestResult::absoluteMediaURL() const
{
#if ENABLE(VIDEO)
    if (RefPtr element = mediaElement()) {
        auto sourceURL = element->currentSrc();
        if (RefPtr page = element->document().page())
            return page->applyLinkDecorationFiltering(sourceURL, LinkDecorationFilteringTrigger::Unspecified);
        return sourceURL;
    }
#endif
    return { };
}

bool HitTestResult::mediaSupportsFullscreen() const
{
#if ENABLE(VIDEO)
    if (RefPtr element = mediaElement())
        return is<HTMLVideoElement>(*element) && element->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModeStandard);
#endif
    return false;
}

#if ENABLE(VIDEO)
HTMLMediaElement* HitTestResult::mediaElement() const
{
    if (!m_innerNonSharedNode)
        return nullptr;

    if (!(m_innerNonSharedNode->renderer() && m_innerNonSharedNode->renderer()->isRenderMedia()))
        return nullptr;

    return dynamicDowncast<HTMLMediaElement>(*m_innerNonSharedNode);
}
#endif

bool HitTestResult::hasMediaElement() const
{
#if ENABLE(VIDEO)
    return !!mediaElement();
#else
    return false;
#endif
}

void HitTestResult::toggleMediaControlsDisplay() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        mediaElt->setControls(!mediaElt->controls());
#endif
}

void HitTestResult::toggleMediaLoopPlayback() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        mediaElt->setLoop(!mediaElt->loop());
#endif
}

void HitTestResult::toggleShowMediaStats() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        mediaElt->setShowingStats(!mediaElt->showingStats());
#endif
}

bool HitTestResult::mediaIsInFullscreen() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElement = this->mediaElement())
        return mediaElement->isVideo() && mediaElement->isStandardFullscreen();
#endif
    return false;
}

void HitTestResult::toggleMediaFullscreenState() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElement = this->mediaElement()) {
        if (mediaElement->isVideo() && mediaElement->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModeStandard)) {
            UserGestureIndicator indicator(IsProcessingUserGesture::Yes, &mediaElement->document());
            mediaElement->toggleStandardFullscreenState();
        }
    }
#endif
}

void HitTestResult::enterFullscreenForVideo() const
{
#if ENABLE(VIDEO)
    RefPtr mediaElement(this->mediaElement());
    if (RefPtr videoElement = dynamicDowncast<HTMLVideoElement>(mediaElement)) {
        if (!videoElement->isFullscreen() && mediaElement->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModeStandard)) {
            UserGestureIndicator indicator(IsProcessingUserGesture::Yes, &mediaElement->document());
            videoElement->webkitEnterFullscreen();
        }
    }
#endif
}

bool HitTestResult::mediaIsInVideoViewer() const
{
#if PLATFORM(MAC) && ENABLE(VIDEO) && ENABLE(VIDEO_PRESENTATION_MODE)
    if (RefPtr mediaElt = mediaElement())
        return is<HTMLVideoElement>(mediaElt) && mediaElt->fullscreenMode() == HTMLMediaElementEnums::VideoFullscreenModeInWindow;
#endif
    return false;
}

void HitTestResult::toggleVideoViewer() const
{
#if PLATFORM(MAC) && ENABLE(VIDEO) && ENABLE(VIDEO_PRESENTATION_MODE)
    RefPtr mediaElement(this->mediaElement());
    RefPtr videoElement = dynamicDowncast<HTMLVideoElement>(mediaElement);
    if (!videoElement || !mediaElement->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModeInWindow))
        return;

    UserGestureIndicator indicator(IsProcessingUserGesture::Yes, &mediaElement->document());
    auto newMode = videoElement->webkitPresentationMode() == HTMLVideoElement::VideoPresentationMode::InWindow ? HTMLVideoElement::VideoPresentationMode::Inline : HTMLVideoElement::VideoPresentationMode::InWindow;

    videoElement->webkitSetPresentationMode(newMode);
#endif
}

bool HitTestResult::mediaControlsEnabled() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElement = this->mediaElement())
        return mediaElement->controls();
#endif
    return false;
}

bool HitTestResult::mediaLoopEnabled() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return mediaElt->loop();
#endif
    return false;
}

bool HitTestResult::mediaStatsShowing() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return mediaElt->showingStats();
#endif
    return false;
}

bool HitTestResult::mediaPlaying() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return !mediaElt->paused();
#endif
    return false;
}

void HitTestResult::toggleMediaPlayState() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        mediaElt->togglePlayState();
#endif
}

bool HitTestResult::mediaHasAudio() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return mediaElt->hasAudio();
#endif
    return false;
}

bool HitTestResult::mediaIsVideo() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return is<HTMLVideoElement>(*mediaElt);
#endif
    return false;
}

bool HitTestResult::mediaMuted() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return mediaElt->muted();
#endif
    return false;
}

void HitTestResult::toggleMediaMuteState() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        mediaElt->setMuted(!mediaElt->muted());
#endif
}

bool HitTestResult::isDownloadableMedia() const
{
#if ENABLE(VIDEO)
    if (RefPtr mediaElt = mediaElement())
        return mediaElt->canSaveMediaData();
#endif

    return false;
}

bool HitTestResult::isOverTextInsideFormControlElement() const
{
    RefPtr element = dynamicDowncast<Element>(innerNode());
    if (!element || !element->isTextField())
        return false;

    auto* frame = element->document().frame();
    if (!frame)
        return false;

    auto framePoint = roundedPointInInnerNodeFrame();
    if (!frame->rangeForPoint(framePoint))
        return false;

    auto position = frame->visiblePositionForPoint(framePoint);
    if (position.isNull())
        return false;

    auto wordRange = enclosingTextUnitOfGranularity(position, TextGranularity::WordGranularity, SelectionDirection::Forward);
    return wordRange && hasAnyPlainText(*wordRange);
}

URL HitTestResult::absoluteLinkURL() const
{
    if (!m_innerURLElement)
        return { };

    auto url = m_innerURLElement->absoluteLinkURL();
    if (RefPtr page = m_innerURLElement->document().page())
        return page->applyLinkDecorationFiltering(url, LinkDecorationFilteringTrigger::Unspecified);

    return url;
}

bool HitTestResult::hasLocalDataForLinkURL() const
{
    auto linkURL = absoluteLinkURL();
    if (linkURL.isEmpty())
        return false;

    if (RefPtr page = m_innerURLElement->document().page())
        return page->hasLocalDataForURL(linkURL);

    return false;
}

bool HitTestResult::isOverLink() const
{
    return m_innerURLElement && m_innerURLElement->isLink();
}

String HitTestResult::titleDisplayString() const
{
    if (!m_innerURLElement)
        return String();
    
    return displayString(m_innerURLElement->title(), m_innerURLElement.get());
}

String HitTestResult::textContent() const
{
    if (!m_innerURLElement)
        return String();
    return m_innerURLElement->textContent();
}

// FIXME: This function needs a better name and may belong in a different class. It's not
// really isContentEditable(); it's more like needsEditingContextMenu(). In many ways, this
// function would make more sense in the ContextMenu class, except that WebElementDictionary 
// hooks into it. Anyway, we should architect this better. 
bool HitTestResult::isContentEditable() const
{
    if (!m_innerNonSharedNode)
        return false;

    if (is<HTMLTextAreaElement>(*m_innerNonSharedNode))
        return true;

    if (RefPtr input = dynamicDowncast<HTMLInputElement>(*m_innerNonSharedNode))
        return input->isTextField();

    return m_innerNonSharedNode->hasEditableStyle();
}

template<typename RectType>
inline HitTestProgress HitTestResult::addNodeToListBasedTestResultCommon(Node* node, const HitTestRequest& request, const HitTestLocation& locationInContainer, const RectType& rect)
{
    // If it is not a list-based hit test, this method has to be no-op.
    if (!request.resultIsElementList()) {
        ASSERT(!isRectBasedTest());
        return HitTestProgress::Stop;
    }

    if (!node)
        return HitTestProgress::Continue;

    if ((request.disallowsUserAgentShadowContent() && node->isInUserAgentShadowTree())
        || (request.disallowsUserAgentShadowContentExceptForImageOverlays() && !ImageOverlay::isInsideOverlay(*node) && node->isInUserAgentShadowTree()))
        node = node->document().ancestorNodeInThisScope(node);

    mutableListBasedTestResult().add(*node);

    if (request.includesAllElementsUnderPoint())
        return HitTestProgress::Continue;

    bool regionFilled = rect.contains(locationInContainer.boundingBox());
    return regionFilled ? HitTestProgress::Stop : HitTestProgress::Continue;
}

HitTestProgress HitTestResult::addNodeToListBasedTestResult(Node* node, const HitTestRequest& request, const HitTestLocation& locationInContainer, const LayoutRect& rect)
{
    return addNodeToListBasedTestResultCommon(node, request, locationInContainer, rect);
}

HitTestProgress HitTestResult::addNodeToListBasedTestResult(Node* node, const HitTestRequest& request, const HitTestLocation& locationInContainer, const FloatRect& rect)
{
    return addNodeToListBasedTestResultCommon(node, request, locationInContainer, rect);
}

void HitTestResult::append(const HitTestResult& other, const HitTestRequest& request)
{
    ASSERT_UNUSED(request, request.resultIsElementList());

    if (!m_innerNode && other.innerNode()) {
        m_innerNode = other.innerNode();
        m_innerNonSharedNode = other.innerNonSharedNode();
        m_localPoint = other.localPoint();
        m_pointInInnerNodeFrame = other.m_pointInInnerNodeFrame;
        m_innerURLElement = other.URLElement();
        m_scrollbar = other.scrollbar();
        m_isOverWidget = other.isOverWidget();
    }

    if (other.m_listBasedTestResult)
        appendToNodeSet(*other.m_listBasedTestResult, mutableListBasedTestResult());
}

const HitTestResult::NodeSet& HitTestResult::listBasedTestResult() const
{
    if (!m_listBasedTestResult)
        m_listBasedTestResult = makeUnique<NodeSet>();
    return *m_listBasedTestResult;
}

HitTestResult::NodeSet& HitTestResult::mutableListBasedTestResult()
{
    if (!m_listBasedTestResult)
        m_listBasedTestResult = makeUnique<NodeSet>();
    return *m_listBasedTestResult;
}

Vector<String> HitTestResult::dictationAlternatives() const
{
    // Return the dictation context handle if the text at this point has DictationAlternative marker, which means this text is
    if (!m_innerNonSharedNode)
        return Vector<String>();

    CheckedPtr markers = m_innerNonSharedNode->document().markersIfExists();
    if (!markers)
        return Vector<String>();

    WeakPtr marker = markers->markerContainingPoint(pointInInnerNodeFrame(), DocumentMarkerType::DictationAlternatives);
    if (!marker)
        return Vector<String>();

    RefPtr frame = innerNonSharedNode()->document().frame();
    if (!frame)
        return Vector<String>();

    return frame->editor().dictationAlternativesForMarker(*marker);
}

RefPtr<Node> HitTestResult::protectedTargetNode() const
{
    return innerNode();
}

Element* HitTestResult::targetElement() const
{
    for (Node* node = m_innerNode.get(); node; node = node->parentInComposedTree()) {
        if (auto* element = dynamicDowncast<Element>(*node))
            return element;
    }
    return nullptr;
}

RefPtr<Element> HitTestResult::protectedTargetElement() const
{
    return targetElement();
}

Element* HitTestResult::innerNonSharedElement() const
{
    auto* node = m_innerNonSharedNode.get();
    if (!node)
        return nullptr;
    if (auto* element = dynamicDowncast<Element>(*node))
        return element;
    return node->parentElement();
}

String HitTestResult::linkSuggestedFilename() const
{
    auto* urlElement = URLElement();
    if (!is<HTMLAnchorElement>(urlElement))
        return nullAtom();
    return ResourceResponse::sanitizeSuggestedFilename(urlElement->attributeWithoutSynchronization(HTMLNames::downloadAttr));
}

bool HitTestResult::mediaSupportsEnhancedFullscreen() const
{
#if PLATFORM(MAC) && ENABLE(VIDEO) && ENABLE(VIDEO_PRESENTATION_MODE)
    if (RefPtr mediaElt = mediaElement())
        return is<HTMLVideoElement>(mediaElt) && mediaElt->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModePictureInPicture);
#endif
    return false;
}

bool HitTestResult::mediaIsInEnhancedFullscreen() const
{
#if PLATFORM(MAC) && ENABLE(VIDEO) && ENABLE(VIDEO_PRESENTATION_MODE)
    if (RefPtr mediaElt = mediaElement())
        return is<HTMLVideoElement>(mediaElt) && mediaElt->fullscreenMode() == HTMLMediaElementEnums::VideoFullscreenModePictureInPicture;
#endif
    return false;
}

void HitTestResult::toggleEnhancedFullscreenForVideo() const
{
#if PLATFORM(MAC) && ENABLE(VIDEO) && ENABLE(VIDEO_PRESENTATION_MODE)
    RefPtr mediaElement(this->mediaElement());
    RefPtr videoElement = dynamicDowncast<HTMLVideoElement>(mediaElement);
    if (!mediaElement || !videoElement || !mediaElement->supportsFullscreen(HTMLMediaElementEnums::VideoFullscreenModePictureInPicture))
        return;

    UserGestureIndicator indicator(IsProcessingUserGesture::Yes, &mediaElement->document());
    if (videoElement->webkitPresentationMode() == HTMLVideoElement::VideoPresentationMode::PictureInPicture)
        videoElement->webkitSetPresentationMode(HTMLVideoElement::VideoPresentationMode::Inline);
    else
        videoElement->webkitSetPresentationMode(HTMLVideoElement::VideoPresentationMode::PictureInPicture);
#endif
}

#if ENABLE(ACCESSIBILITY_ANIMATION_CONTROL)
HTMLImageElement* HitTestResult::imageElement() const
{
    if (auto* imageElement = dynamicDowncast<HTMLImageElement>(m_innerNonSharedNode.get()))
        return imageElement;
    return nullptr;
}

bool HitTestResult::isAnimating() const
{
    if (auto* imageElement = this->imageElement())
        return imageElement->allowsAnimation();
    return false;
}

void HitTestResult::playAnimation() const
{
    setAllowsAnimation(true);
}

void HitTestResult::pauseAnimation() const
{
    setAllowsAnimation(false);
}

void HitTestResult::setAllowsAnimation(bool allowAnimation) const
{
    if (auto* imageElement = this->imageElement()) {
        imageElement->setAllowsAnimation(allowAnimation);
        if (auto* renderer = m_innerNonSharedNode->renderer())
            renderer->repaint();
    }
}
#endif // ENABLE(ACCESSIBILITY_ANIMATION_CONTROL)

} // namespace WebCore
